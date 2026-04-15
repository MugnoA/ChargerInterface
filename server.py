"""
DC/DC Bidirectional Converter — Backend Server
================================================
WebSocket server that bridges the web dashboard to the F28M35x
microcontroller via XCP-on-Serial, or runs in simulation mode
for development and presentations.

Usage:
  python server.py --sim                    # Simulation mode (no hardware)
  python server.py --port COM3              # Real hardware on COM3
  python server.py --port COM3 --baud 115200

Dependencies:
  pip install websockets pyserial

For real XCP communication (optional, advanced):
  pip install pyxcp
"""

import argparse
import asyncio
import json
import logging
import math
import struct
import time
from typing import Any

import websockets

# ─── Logging ────────────────────────────────────────────────

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(message)s",
    datefmt="%H:%M:%S",
)
log = logging.getLogger("dcdc-server")


# ═══════════════════════════════════════════════════════════════
#  MEMORY MAP — mirrors the Simulink-generated structs
# ═══════════════════════════════════════════════════════════════
# These addresses must match your .map file (tet.map).
# Run `grep "tet_P\|tet_B\|tet_DW" tet.map` to find them.
# The offsets below are based on the P_tet_T struct field order
# from tet_data.c. Adjust BASE_ADDR after checking tet.map.

BASE_ADDR_P = 0x00000000   # TODO: set from tet.map -> tet_P symbol
BASE_ADDR_B = 0x00000000   # TODO: set from tet.map -> tet_B symbol

# Parameter offsets within P_tet_T (each field is real_T = 8 bytes on C28x double,
# but C2000 uses 32-bit IEEE 754 floats for real_T when configured as float.
# Check your tet.h for the actual typedef. Commonly real_T = double = 64-bit.)
# Using 64-bit (8-byte) offsets here:
PARAM_MAP = {
    "CurrentSetPoint":    {"offset": 0,   "size": 8, "fmt": "<d"},
    "DigitalCurrentConvert": {"offset": 8,   "size": 8, "fmt": "<d"},
    "DigitalCurrentSlope": {"offset": 16,  "size": 8, "fmt": "<d"},
    "DigitalPotenciometer": {"offset": 24,  "size": 8, "fmt": "<d"},
    "IntegralGain":        {"offset": 32,  "size": 8, "fmt": "<d"},
    "OffsetCurrentValue":  {"offset": 40,  "size": 8, "fmt": "<d"},
    "OffsetValue":         {"offset": 48,  "size": 8, "fmt": "<d"},
    "ProportionalGain":    {"offset": 56,  "size": 8, "fmt": "<d"},
    "PulseWidth":          {"offset": 64,  "size": 8, "fmt": "<d"},
    "Slope":               {"offset": 72,  "size": 8, "fmt": "<d"},
    "TimerPeriod":         {"offset": 80,  "size": 8, "fmt": "<d"},
    "VoltageSetPoint":     {"offset": 88,  "size": 8, "fmt": "<d"},
}

# Signals (B_tet_T) — the ADC readings
SIGNAL_MAP = {
    "ADC":  {"offset": 0, "size": 2, "fmt": "<H"},  # uint16 — output voltage ADC
    "ADC1": {"offset": 2, "size": 2, "fmt": "<H"},  # uint16 — output current ADC
    # TODO: add input ADC offsets after updating Simulink model
    # "ADC_IN_V": {"offset": X, "size": 2, "fmt": "<H"},  # input voltage ADC
    # "ADC_IN_I": {"offset": Y, "size": 2, "fmt": "<H"},  # input current ADC
}


# ═══════════════════════════════════════════════════════════════
#  XCP DRIVER — real hardware communication
# ═══════════════════════════════════════════════════════════════

class XCPDriver:
    """
    Communicates with the F28M35x XCP slave over serial.

    This uses raw XCP commands over the rtiostream serial framing
    that Simulink generates. For a more robust approach, use pyXCP
    with SxI transport and your A2L file.

    The Simulink XCP-on-Serial implementation uses a simple framing:
      [LEN_LO] [LEN_HI] [COUNTER] [XCP_PAYLOAD...]
    where LEN is the total packet length including the 2-byte header.
    """

    # XCP Command codes
    CMD_CONNECT        = 0xFF
    CMD_DISCONNECT     = 0xFE
    CMD_GET_STATUS     = 0xFD
    CMD_SHORT_UPLOAD   = 0xF4
    CMD_DOWNLOAD       = 0xF0
    CMD_SET_MTA        = 0xF6
    RES_OK             = 0xFF
    RES_ERR            = 0xFE

    def __init__(self, port: str, baud: int = 115200):
        import serial
        self.ser = serial.Serial(
            port=port,
            baudrate=baud,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=1.0,
        )
        self.counter = 0
        self.connected = False
        log.info(f"Serial port {port} opened at {baud} baud")

    def _next_counter(self) -> int:
        c = self.counter
        self.counter = (self.counter + 1) & 0xFFFF
        return c

    def _send_packet(self, payload: bytes):
        """Send an XCP packet with rtiostream-style framing."""
        length = len(payload) + 2  # +2 for counter
        counter = self._next_counter()
        frame = struct.pack("<HH", length, counter) + payload
        self.ser.write(frame)
        self.ser.flush()

    def _recv_packet(self, timeout: float = 1.0) -> bytes:
        """Receive an XCP response packet."""
        self.ser.timeout = timeout
        header = self.ser.read(4)  # LEN(2) + COUNTER(2)
        if len(header) < 4:
            raise TimeoutError("No response from target")
        length = struct.unpack("<H", header[0:2])[0]
        payload_len = length - 2
        if payload_len > 0:
            payload = self.ser.read(payload_len)
        else:
            payload = b""
        return payload

    def connect(self):
        """Send XCP CONNECT command."""
        self._send_packet(bytes([self.CMD_CONNECT, 0x00]))
        resp = self._recv_packet()
        if resp and resp[0] == self.RES_OK:
            self.connected = True
            log.info("XCP connected to target")
        else:
            raise ConnectionError(f"XCP CONNECT failed: {resp.hex() if resp else 'no response'}")

    def disconnect(self):
        """Send XCP DISCONNECT command."""
        if self.connected:
            self._send_packet(bytes([self.CMD_DISCONNECT]))
            self.connected = False
            log.info("XCP disconnected")

    def short_upload(self, address: int, ext: int, size: int) -> bytes:
        """Read `size` bytes from target memory at `address`."""
        payload = struct.pack("<BBBBI", self.CMD_SHORT_UPLOAD, size, 0, ext, address)
        self._send_packet(payload)
        resp = self._recv_packet()
        if resp and resp[0] == self.RES_OK:
            return resp[1:1+size]
        raise IOError(f"SHORT_UPLOAD failed at 0x{address:08X}")

    def download(self, address: int, ext: int, data: bytes):
        """Write `data` to target memory at `address`."""
        # First SET_MTA
        mta_payload = struct.pack("<BBBI", self.CMD_SET_MTA, 0, ext, address)
        self._send_packet(mta_payload)
        resp = self._recv_packet()
        if not resp or resp[0] != self.RES_OK:
            raise IOError(f"SET_MTA failed at 0x{address:08X}")

        # Then DOWNLOAD
        dl_payload = bytes([self.CMD_DOWNLOAD, len(data)]) + data
        self._send_packet(dl_payload)
        resp = self._recv_packet()
        if not resp or resp[0] != self.RES_OK:
            raise IOError(f"DOWNLOAD failed at 0x{address:08X}")

    def read_param(self, name: str) -> float:
        """Read a named parameter from tet_P."""
        info = PARAM_MAP[name]
        raw = self.short_upload(
            BASE_ADDR_P + info["offset"], 0, info["size"]
        )
        return struct.unpack(info["fmt"], raw)[0]

    def write_param(self, name: str, value: float):
        """Write a named parameter to tet_P."""
        info = PARAM_MAP[name]
        data = struct.pack(info["fmt"], value)
        self.download(BASE_ADDR_P + info["offset"], 0, data)

    def read_signal(self, name: str):
        """Read a named signal from tet_B."""
        info = SIGNAL_MAP[name]
        raw = self.short_upload(
            BASE_ADDR_B + info["offset"], 0, info["size"]
        )
        return struct.unpack(info["fmt"], raw)[0]

    def close(self):
        self.disconnect()
        self.ser.close()


# ═══════════════════════════════════════════════════════════════
#  SIMULATION DRIVER — for development without hardware
# ═══════════════════════════════════════════════════════════════

class SimDriver:
    """
    Simulates the tet_step() function from tet.c for dashboard
    development and presentation demos without the real hardware.
    """

    def __init__(self):
        self.params = {
            "CurrentSetPoint": 4.0,
            "DigitalCurrentConvert": 0.00080586080586080586,
            "DigitalCurrentSlope": 0.4,
            "DigitalPotenciometer": 0.00080586080586080586,
            "IntegralGain": 10.0,
            "OffsetCurrentValue": -1.26,
            "OffsetValue": 0.0,
            "ProportionalGain": 0.1,
            "PulseWidth": 750.0,
            "Slope": 0.0,
            "TimerPeriod": 1499.0,
            "VoltageSetPoint": 250.0,
            "Constant2_Value": 1.0,
            "Integrator_gainval": 0.0005,
            "Integrator_gainval_a": 0.0005,
            "Gain_Gain": 4.166666666666667,
            "UpperSatI": 25.0,
            "LowerSatI": 0.0,
            "UpperSatV": 25.0,
            "LowerSatV": 0.0,
        }
        self.running = False
        self.integrator_i = 0.0
        self.integrator_v = 0.0
        self.sim_time = 0.0

        # Last computed values
        self.voltage_in = 0.0
        self.current_in = 0.0
        self.voltage_out = 0.0
        self.current_out = 0.0
        self.pi_output = 0.0
        self.phase_reg = 0.0

    def connect(self):
        log.info("Simulation mode — no hardware connected")

    def disconnect(self):
        pass

    def close(self):
        pass

    def read_param(self, name: str) -> float:
        return self.params.get(name, 0.0)

    def write_param(self, name: str, value: float):
        self.params[name] = value
        log.debug(f"Param {name} = {value}")

    def step(self):
        """Execute one simulation step (mirrors tet_step logic)."""
        if not self.running:
            return

        P = self.params
        self.sim_time += 0.001

        # Input side (simulated DC source with ripple)
        self.voltage_in = 48 + 2 * math.sin(self.sim_time * 0.3) + 0.5 * (math.sin(self.sim_time * 17.3) * 0.5)
        self.current_in = 8 + 1.5 * math.sin(self.sim_time * 0.25) + 0.3 * (math.sin(self.sim_time * 23.7) * 0.5)

        # Output side ADC values (noisy sine waves)
        adc_v = 1550 + 180 * math.sin(self.sim_time * 0.4) + 25 * (math.sin(self.sim_time * 17.3) * 0.5)
        adc_i = 2500 + 120 * math.sin(self.sim_time * 0.25) + 15 * (math.sin(self.sim_time * 23.7) * 0.5)

        # Output current measurement chain
        self.current_out = (
            (adc_i * P["DigitalCurrentConvert"] + P["OffsetCurrentValue"])
            * P["DigitalCurrentSlope"]
        )

        # Output voltage measurement chain
        self.voltage_out = (
            adc_v * P["DigitalPotenciometer"] + P["Slope"] + P["OffsetValue"]
        )

        # Current PI
        error_i = P["CurrentSetPoint"] - self.current_out
        int_term_i = P["IntegralGain"] * error_i * P["Integrator_gainval"]
        self.integrator_i += int_term_i

        # Voltage PI
        error_v = P["VoltageSetPoint"] - self.voltage_out
        int_term_v = P["IntegralGain"] * error_v * P["Integrator_gainval_a"]
        self.integrator_v += int_term_v

        # Switch: current or voltage control
        if P["Constant2_Value"] > 0:
            raw = (error_i + self.integrator_i) * P["ProportionalGain"]
            self.pi_output = max(P["LowerSatI"], min(P["UpperSatI"], raw))
        else:
            raw = (error_v + self.integrator_v) * P["ProportionalGain"]
            self.pi_output = max(P["LowerSatV"], min(P["UpperSatV"], raw))

        # Phase shift register value
        self.phase_reg = self.pi_output * P["Gain_Gain"]

    def get_state(self) -> dict:
        """Return the full state snapshot."""
        P = self.params

        return {
            "voltageIn": round(self.voltage_in, 2),
            "currentIn": round(self.current_in, 3),
            "voltageOut": round(self.voltage_out, 2),
            "currentOut": round(self.current_out, 3),
            "piOutput": round(self.pi_output, 3),
            "phaseReg": round(self.phase_reg, 2),
            "running": self.running,
            "mode": "current" if P["Constant2_Value"] > 0 else "voltage",
            "params": {
                "CurrentSetPoint": P["CurrentSetPoint"],
                "VoltageSetPoint": P["VoltageSetPoint"],
                "ProportionalGain": P["ProportionalGain"],
                "IntegralGain": P["IntegralGain"],
                "PulseWidth": P["PulseWidth"],
                "TimerPeriod": P["TimerPeriod"],
            },
        }


# ═══════════════════════════════════════════════════════════════
#  WEBSOCKET SERVER
# ═══════════════════════════════════════════════════════════════

class DashboardServer:
    """
    WebSocket server that bridges dashboard ↔ driver.

    Protocol (JSON messages):

    Dashboard → Server:
      {"cmd": "start"}
      {"cmd": "stop"}
      {"cmd": "setMode", "mode": "current"|"voltage"}
      {"cmd": "setParam", "name": "ProportionalGain", "value": 0.5}
      {"cmd": "getState"}

    Server → Dashboard:
      {"type": "state", ...state_data}
      {"type": "ack", "cmd": "...", "ok": true}
      {"type": "error", "message": "..."}
    """

    def __init__(self, driver, host: str = "0.0.0.0", ws_port: int = 8765):
        self.driver = driver
        self.host = host
        self.ws_port = ws_port
        self.clients: set = set()
        self.stream_task = None

    async def handler(self, websocket):
        """Handle a single WebSocket connection."""
        self.clients.add(websocket)
        remote = websocket.remote_address
        log.info(f"Client connected: {remote}")

        try:
            async for raw_message in websocket:
                try:
                    msg = json.loads(raw_message)
                    await self.handle_message(websocket, msg)
                except json.JSONDecodeError:
                    await websocket.send(json.dumps({
                        "type": "error",
                        "message": "Invalid JSON"
                    }))
        except websockets.exceptions.ConnectionClosed:
            pass
        finally:
            self.clients.discard(websocket)
            log.info(f"Client disconnected: {remote}")

    async def handle_message(self, ws, msg: dict):
        """Route incoming commands."""
        cmd = msg.get("cmd")

        if cmd == "start":
            if isinstance(self.driver, SimDriver):
                self.driver.running = True
                self.driver.integrator_i = 0.0
                self.driver.integrator_v = 0.0
            await ws.send(json.dumps({"type": "ack", "cmd": "start", "ok": True}))
            log.info("Converter STARTED")

        elif cmd == "stop":
            if isinstance(self.driver, SimDriver):
                self.driver.running = False
            await ws.send(json.dumps({"type": "ack", "cmd": "stop", "ok": True}))
            log.info("Converter STOPPED")

        elif cmd == "setMode":
            mode = msg.get("mode", "current")
            val = 1.0 if mode == "current" else 0.0
            self.driver.write_param("Constant2_Value", val)
            # Reset integrators on mode change
            if isinstance(self.driver, SimDriver):
                self.driver.integrator_i = 0.0
                self.driver.integrator_v = 0.0
            await ws.send(json.dumps({"type": "ack", "cmd": "setMode", "ok": True, "mode": mode}))
            log.info(f"Mode → {mode}")

        elif cmd == "setParam":
            name = msg.get("name")
            value = msg.get("value")
            if name and value is not None:
                try:
                    self.driver.write_param(name, float(value))
                    await ws.send(json.dumps({
                        "type": "ack", "cmd": "setParam", "ok": True,
                        "name": name, "value": float(value)
                    }))
                except Exception as e:
                    await ws.send(json.dumps({
                        "type": "error",
                        "message": f"Failed to set {name}: {e}"
                    }))
            else:
                await ws.send(json.dumps({
                    "type": "error",
                    "message": "setParam requires 'name' and 'value'"
                }))

        elif cmd == "getState":
            state = self._build_state()
            await ws.send(json.dumps({"type": "state", **state}))

        else:
            await ws.send(json.dumps({
                "type": "error",
                "message": f"Unknown command: {cmd}"
            }))

    def _build_state(self) -> dict:
        """Build state snapshot from driver."""
        if isinstance(self.driver, SimDriver):
            return self.driver.get_state()
        else:
            # Real XCP driver — read values from target
            try:
                adc_v = self.driver.read_signal("ADC")
                adc_i = self.driver.read_signal("ADC1")
                # TODO: read input ADCs when added to Simulink model
                # adc_v_in = self.driver.read_signal("ADC_IN_V")
                # adc_i_in = self.driver.read_signal("ADC_IN_I")
                P = {}
                for name in PARAM_MAP:
                    P[name] = self.driver.read_param(name)

                # Reconstruct output measurements (same as tet_step)
                current_out = (adc_i * P["DigitalCurrentConvert"] + P["OffsetCurrentValue"]) * P["DigitalCurrentSlope"]
                voltage_out = adc_v * P["DigitalPotenciometer"] + P["Slope"] + P["OffsetValue"]

                return {
                    "voltageIn": 0,   # TODO: compute from ADC_IN_V
                    "currentIn": 0,   # TODO: compute from ADC_IN_I
                    "voltageOut": round(voltage_out, 2),
                    "currentOut": round(current_out, 3),
                    "piOutput": 0,
                    "phaseReg": 0,
                    "running": True,
                    "mode": "current",
                    "params": {
                        "CurrentSetPoint": P.get("CurrentSetPoint", 0),
                        "VoltageSetPoint": P.get("VoltageSetPoint", 0),
                        "ProportionalGain": P.get("ProportionalGain", 0),
                        "IntegralGain": P.get("IntegralGain", 0),
                        "PulseWidth": P.get("PulseWidth", 0),
                        "TimerPeriod": P.get("TimerPeriod", 0),
                    },
                }
            except Exception as e:
                log.error(f"XCP read failed: {e}")
                return {"error": str(e)}

    async def stream_loop(self):
        """Periodic loop: run simulation steps and broadcast state."""
        while True:
            if isinstance(self.driver, SimDriver):
                # Run 5 sim steps per broadcast cycle
                for _ in range(5):
                    self.driver.step()

            if self.clients:
                state = self._build_state()
                message = json.dumps({"type": "state", **state})
                # Broadcast to all connected clients
                disconnected = set()
                for ws in self.clients:
                    try:
                        await ws.send(message)
                    except websockets.exceptions.ConnectionClosed:
                        disconnected.add(ws)
                self.clients -= disconnected

            await asyncio.sleep(0.05)  # 20 Hz update rate

    async def run(self):
        """Start the WebSocket server and streaming loop."""
        self.driver.connect()

        async with websockets.serve(self.handler, self.host, self.ws_port):
            log.info(f"WebSocket server running on ws://{self.host}:{self.ws_port}")
            log.info("Open index.html in your browser to connect")
            await self.stream_loop()


# ═══════════════════════════════════════════════════════════════
#  MAIN
# ═══════════════════════════════════════════════════════════════

def main():
    parser = argparse.ArgumentParser(
        description="DC/DC Converter Dashboard — Backend Server"
    )
    parser.add_argument(
        "--sim", action="store_true",
        help="Run in simulation mode (no hardware required)"
    )
    parser.add_argument(
        "--port", type=str, default=None,
        help="Serial port for XCP communication (e.g., COM3, /dev/ttyUSB0)"
    )
    parser.add_argument(
        "--baud", type=int, default=115200,
        help="Serial baud rate (default: 115200)"
    )
    parser.add_argument(
        "--ws-port", type=int, default=8765,
        help="WebSocket server port (default: 8765)"
    )
    parser.add_argument(
        "--host", type=str, default="0.0.0.0",
        help="WebSocket bind address (default: 0.0.0.0)"
    )
    args = parser.parse_args()

    if args.sim:
        driver = SimDriver()
    elif args.port:
        driver = XCPDriver(port=args.port, baud=args.baud)
    else:
        log.warning("No --port specified, defaulting to --sim mode")
        driver = SimDriver()

    server = DashboardServer(driver, host=args.host, ws_port=args.ws_port)

    try:
        asyncio.run(server.run())
    except KeyboardInterrupt:
        log.info("Shutting down...")
    finally:
        driver.close()


if __name__ == "__main__":
    main()
