/* ============================================
   DC/DC Bidirectional Converter — app.js
   4-chart layout: input (V,I) | output (V,I)
   ============================================ */

const WS_URL = "ws://localhost:8765";
const RECONNECT_DELAY = 3000;

let ws = null, wsConnected = false, localSim = false;

// ── Simulation params ──────────────────────
const P = {
  CurrentSetPoint: 4.0,
  DigitalCurrentConvert: 0.00080586080586080586,
  DigitalCurrentSlope: 0.4,
  DigitalPotenciometer: 0.00080586080586080586,
  IntegralGain: 10.0,
  OffsetCurrentValue: -1.26,
  OffsetValue: 0.0,
  ProportionalGain: 0.1,
  PulseWidth: 750.0,
  Slope: 0.0,
  TimerPeriod: 1499.0,
  VoltageSetPoint: 250.0,
  Constant2_Value: 1.0,
  Integrator_gainval: 0.0005,
  Integrator_gainval_a: 0.0005,
  Gain_Gain: 4.166666666666667,
  UpperSatI: 25.0, LowerSatI: 0.0,
  UpperSatV: 25.0, LowerSatV: 0.0,
};

let running = false, IntegratorI = 0, IntegratorV = 0, simTime = 0;
let lastVoltIn = 0, lastCurrIn = 0, lastVoltOut = 0, lastCurrOut = 0;
let lastPI = 0, lastPhaseReg = 0;

const HIST = 250;
const hVoltIn  = new Array(HIST).fill(0);
const hCurrIn  = new Array(HIST).fill(0);
const hVoltOut = new Array(HIST).fill(0);
const hCurrOut = new Array(HIST).fill(0);

// ── DOM ────────────────────────────────────
const dom = {};

function cacheDom() {
  dom.statusPill = document.getElementById("statusPill");
  dom.statusText = document.getElementById("statusText");
  dom.powerBtn   = document.getElementById("powerBtn");
  dom.modeI      = document.getElementById("modeI");
  dom.modeV      = document.getElementById("modeV");
  dom.tuneBtn    = document.getElementById("tuneBtn");
  dom.drawer     = document.getElementById("drawer");
  dom.overlay    = document.getElementById("drawerOverlay");
  dom.mainContent= document.getElementById("mainContent");
  dom.connBadge  = document.getElementById("connBadge");

  dom.mVoltIn  = document.getElementById("mVoltIn");
  dom.mCurrIn  = document.getElementById("mCurrIn");
  dom.mVoltOut = document.getElementById("mVoltOut");
  dom.mCurrOut = document.getElementById("mCurrOut");
  dom.mPowerIn = document.getElementById("mPowerIn");
  dom.mPowerOut= document.getElementById("mPowerOut");
  dom.mPhase   = document.getElementById("mPhase");

  dom.spCurrVal = document.getElementById("spCurrVal");
  dom.spVoltVal = document.getElementById("spVoltVal");
  dom.spCurrTag = document.getElementById("spCurrTag");
  dom.spVoltTag = document.getElementById("spVoltTag");

  dom.cvVoltIn  = document.getElementById("cvVoltIn");
  dom.cvCurrIn  = document.getElementById("cvCurrIn");
  dom.cvVoltOut = document.getElementById("cvVoltOut");
  dom.cvCurrOut = document.getElementById("cvCurrOut");
  dom.cvPWM     = document.getElementById("cvPWM");
}

// ── WebSocket ──────────────────────────────

function connectWebSocket() {
  try { ws = new WebSocket(WS_URL); } catch (e) { enterLocalMode(); return; }
  ws.onopen = () => { wsConnected = true; localSim = false; updateConnBadge(); };
  ws.onmessage = (ev) => {
    try {
      const msg = JSON.parse(ev.data);
      if (msg.type === "state") handleServerState(msg);
    } catch (e) {}
  };
  ws.onclose = () => {
    wsConnected = false; ws = null; updateConnBadge();
    setTimeout(() => { if (!wsConnected) connectWebSocket(); }, RECONNECT_DELAY);
  };
  ws.onerror = () => { if (!wsConnected) enterLocalMode(); };
}

function enterLocalMode() { localSim = true; wsConnected = false; updateConnBadge(); }
function wsSend(obj) { if (ws && ws.readyState === WebSocket.OPEN) ws.send(JSON.stringify(obj)); }

function updateConnBadge() {
  if (!dom.connBadge) return;
  if (wsConnected)   { dom.connBadge.textContent = "LIVE";    dom.connBadge.className = "conn-badge live"; }
  else if (localSim) { dom.connBadge.textContent = "SIM";     dom.connBadge.className = "conn-badge sim"; }
  else               { dom.connBadge.textContent = "OFFLINE";  dom.connBadge.className = "conn-badge offline"; }
}

// ── Handle server state ────────────────────

function handleServerState(s) {
  lastVoltIn  = s.voltageIn  || 0;
  lastCurrIn  = s.currentIn  || 0;
  lastVoltOut = s.voltageOut || 0;
  lastCurrOut = s.currentOut || 0;
  lastPI      = s.piOutput   || 0;
  lastPhaseReg= s.phaseReg   || 0;
  running     = s.running    || false;

  hVoltIn.push(lastVoltIn);   if (hVoltIn.length > HIST) hVoltIn.shift();
  hCurrIn.push(lastCurrIn);   if (hCurrIn.length > HIST) hCurrIn.shift();
  hVoltOut.push(lastVoltOut);  if (hVoltOut.length > HIST) hVoltOut.shift();
  hCurrOut.push(lastCurrOut);  if (hCurrOut.length > HIST) hCurrOut.shift();

  updateUI(s);
  renderCharts();
}

function updateUI(s) {
  dom.mVoltIn.textContent  = lastVoltIn.toFixed(1);
  dom.mCurrIn.textContent  = lastCurrIn.toFixed(2);
  dom.mVoltOut.textContent = lastVoltOut.toFixed(1);
  dom.mCurrOut.textContent = lastCurrOut.toFixed(2);
  dom.mPowerIn.textContent = (lastVoltIn * lastCurrIn).toFixed(1);
  dom.mPowerOut.textContent= (lastVoltOut * lastCurrOut).toFixed(1);

  const phaseDeg = P.TimerPeriod > 0 ? (lastPhaseReg / P.TimerPeriod) * 360 : 0;
  dom.mPhase.textContent = phaseDeg.toFixed(1);

  if (running) {
    dom.statusPill.className = "status-pill running";
    dom.statusText.textContent = "RUNNING";
    dom.powerBtn.textContent = "Apagar";
    dom.powerBtn.className = "btn btn-stop";
  } else {
    dom.statusPill.className = "status-pill stopped";
    dom.statusText.textContent = "STOPPED";
    dom.powerBtn.textContent = "Encender";
    dom.powerBtn.className = "btn btn-start";
  }

  if (s && s.mode) {
    dom.modeI.className = s.mode === "current" ? "mode-btn active" : "mode-btn";
    dom.modeV.className = s.mode === "voltage" ? "mode-btn active" : "mode-btn";
    updateActiveTag(s.mode === "current" ? 1 : 0);
  }
  if (s && s.params) {
    dom.spCurrVal.textContent = (s.params.CurrentSetPoint || 0).toFixed(1);
    dom.spVoltVal.textContent = (s.params.VoltageSetPoint || 0).toFixed(0);
  }
}

// ── Power / mode ───────────────────────────

function togglePower() {
  if (wsConnected) { wsSend({ cmd: running ? "stop" : "start" }); return; }
  running = !running;
  if (running) { IntegratorI = 0; IntegratorV = 0; }
  updateUI(null);
}

function setMode(mode) {
  if (wsConnected) { wsSend({ cmd: "setMode", mode: mode === 1 ? "current" : "voltage" }); return; }
  P.Constant2_Value = mode; IntegratorI = 0; IntegratorV = 0;
  dom.modeI.className = mode === 1 ? "mode-btn active" : "mode-btn";
  dom.modeV.className = mode === 0 ? "mode-btn active" : "mode-btn";
  updateActiveTag(mode);
}

function updateActiveTag(mode) {
  if (mode === undefined) mode = P.Constant2_Value;
  if (mode > 0) {
    dom.spCurrTag.classList.add("active-tag"); dom.spVoltTag.classList.remove("active-tag");
    dom.spCurrTag.textContent = "Activo"; dom.spVoltTag.textContent = "Inactivo";
  } else {
    dom.spVoltTag.classList.add("active-tag"); dom.spCurrTag.classList.remove("active-tag");
    dom.spVoltTag.textContent = "Activo"; dom.spCurrTag.textContent = "Inactivo";
  }
}

// ── Drawer ─────────────────────────────────

let drawerOpen = false;
function toggleDrawer() {
  drawerOpen = !drawerOpen;
  dom.drawer.classList.toggle("open", drawerOpen);
  dom.overlay.classList.toggle("visible", drawerOpen);
  dom.mainContent.classList.toggle("drawer-open", drawerOpen);
  dom.tuneBtn.classList.toggle("active", drawerOpen);
}
function closeDrawer() {
  drawerOpen = false;
  dom.drawer.classList.remove("open"); dom.overlay.classList.remove("visible");
  dom.mainContent.classList.remove("drawer-open"); dom.tuneBtn.classList.remove("active");
}

// ── Param binding ──────────────────────────

function bindParam(sliderId, inputId, displayId, paramKey, fmt) {
  const slider = document.getElementById(sliderId);
  const input  = document.getElementById(inputId);
  const disp   = document.getElementById(displayId);
  function update(val) {
    slider.value = val; input.value = val;
    if (disp) disp.textContent = fmt ? fmt(val) : val;
    if (wsConnected) wsSend({ cmd: "setParam", name: paramKey, value: val });
    else P[paramKey] = val;
    if (paramKey === "CurrentSetPoint") dom.spCurrVal.textContent = val.toFixed(1);
    if (paramKey === "VoltageSetPoint") dom.spVoltVal.textContent = val.toFixed(0);
  }
  slider.addEventListener("input", () => update(parseFloat(slider.value)));
  input.addEventListener("change", () => {
    let v = Math.max(parseFloat(slider.min), Math.min(parseFloat(slider.max), parseFloat(input.value)));
    update(v);
  });
}

// ── Local simulation ───────────────────────

function clamp(v, lo, hi) { return Math.max(lo, Math.min(hi, v)); }

function localSimStep() {
  if (!running || !localSim) return;
  simTime += 0.001;

  // Input side (simulated source)
  lastVoltIn = 48 + 2 * Math.sin(simTime * 0.3) + (Math.random() - 0.5) * 0.5;
  lastCurrIn = 8 + 1.5 * Math.sin(simTime * 0.25) + (Math.random() - 0.5) * 0.3;

  // Output side ADC readings
  const adcV = 1550 + 180 * Math.sin(simTime * 0.4) + (Math.random() - 0.5) * 25;
  const adcI = 2500 + 120 * Math.sin(simTime * 0.25) + (Math.random() - 0.5) * 15;

  lastCurrOut = (adcI * P.DigitalCurrentConvert + P.OffsetCurrentValue) * P.DigitalCurrentSlope;
  lastVoltOut = adcV * P.DigitalPotenciometer + P.Slope + P.OffsetValue;

  // PI controllers
  const errorI = P.CurrentSetPoint - lastCurrOut;
  IntegratorI += P.IntegralGain * errorI * P.Integrator_gainval;
  const errorV = P.VoltageSetPoint - lastVoltOut;
  IntegratorV += P.IntegralGain * errorV * P.Integrator_gainval_a;

  if (P.Constant2_Value > 0)
    lastPI = clamp((errorI + IntegratorI) * P.ProportionalGain, P.LowerSatI, P.UpperSatI);
  else
    lastPI = clamp((errorV + IntegratorV) * P.ProportionalGain, P.LowerSatV, P.UpperSatV);

  lastPhaseReg = lastPI * P.Gain_Gain;

  hVoltIn.push(lastVoltIn);   if (hVoltIn.length > HIST) hVoltIn.shift();
  hCurrIn.push(lastCurrIn);   if (hCurrIn.length > HIST) hCurrIn.shift();
  hVoltOut.push(lastVoltOut);  if (hVoltOut.length > HIST) hVoltOut.shift();
  hCurrOut.push(lastCurrOut);  if (hCurrOut.length > HIST) hCurrOut.shift();

  dom.mVoltIn.textContent  = lastVoltIn.toFixed(1);
  dom.mCurrIn.textContent  = lastCurrIn.toFixed(2);
  dom.mVoltOut.textContent = lastVoltOut.toFixed(1);
  dom.mCurrOut.textContent = lastCurrOut.toFixed(2);
  dom.mPowerIn.textContent = (lastVoltIn * lastCurrIn).toFixed(1);
  dom.mPowerOut.textContent= (lastVoltOut * lastCurrOut).toFixed(1);
  dom.mPhase.textContent   = P.TimerPeriod > 0 ? ((lastPhaseReg / P.TimerPeriod) * 360).toFixed(1) : "0.0";
}

// ── Canvas rendering ───────────────────────

function setupCanvas(canvas) {
  const rect = canvas.getBoundingClientRect();
  const dpr = window.devicePixelRatio || 1;
  canvas.width = rect.width * dpr;
  canvas.height = rect.height * dpr;
  const ctx = canvas.getContext("2d");
  ctx.scale(dpr, dpr);
  return { ctx, w: rect.width, h: rect.height };
}

function drawWaveform(canvas, data, color, glowColor) {
  const { ctx, w, h } = setupCanvas(canvas);
  ctx.clearRect(0, 0, w, h);

  ctx.strokeStyle = "rgba(255,255,255,0.04)"; ctx.lineWidth = 0.5;
  for (let i = 0; i < 5; i++) { const y = h*0.08+i*(h*0.84/4); ctx.beginPath(); ctx.moveTo(0,y); ctx.lineTo(w,y); ctx.stroke(); }
  for (let i = 0; i < 8; i++) { const x = w*(i/7); ctx.beginPath(); ctx.moveTo(x,0); ctx.lineTo(x,h); ctx.stroke(); }

  if (data.length < 2) return;
  let dMin = Infinity, dMax = -Infinity;
  for (let i = 0; i < data.length; i++) { if (data[i]<dMin) dMin=data[i]; if (data[i]>dMax) dMax=data[i]; }
  const pad = (dMax-dMin)*0.12||1; dMin-=pad; dMax+=pad;
  const range = dMax-dMin||1;

  ctx.strokeStyle = glowColor; ctx.lineWidth = 6; ctx.lineCap="round"; ctx.lineJoin="round";
  ctx.beginPath();
  for (let i=0;i<data.length;i++) { const x=(i/(data.length-1))*w, y=h*0.92-((data[i]-dMin)/range)*(h*0.84); i===0?ctx.moveTo(x,y):ctx.lineTo(x,y); }
  ctx.stroke();

  ctx.strokeStyle = color; ctx.lineWidth = 1.8;
  ctx.beginPath();
  for (let i=0;i<data.length;i++) { const x=(i/(data.length-1))*w, y=h*0.92-((data[i]-dMin)/range)*(h*0.84); i===0?ctx.moveTo(x,y):ctx.lineTo(x,y); }
  ctx.stroke();

  ctx.fillStyle = "rgba(255,255,255,0.3)"; ctx.font = "10px JetBrains Mono, monospace";
  ctx.fillText(dMax.toFixed(1), 4, h*0.08+12);
  ctx.fillText(dMin.toFixed(1), 4, h*0.92);
  ctx.fillText(((dMax+dMin)/2).toFixed(1), 4, h*0.5+4);
}

function drawPWM(canvas) {
  const { ctx, w, h } = setupCanvas(canvas);
  ctx.clearRect(0,0,w,h);
  const duty = P.PulseWidth / P.TimerPeriod;
  const periods = 5, pw = w / periods;

  ctx.strokeStyle = "rgba(255,255,255,0.04)"; ctx.lineWidth = 0.5;
  for (let i=0;i<=periods;i++) { ctx.beginPath(); ctx.moveTo(i*pw,0); ctx.lineTo(i*pw,h); ctx.stroke(); }

  ctx.strokeStyle = "#4A9EF5"; ctx.lineWidth = 2;
  ctx.beginPath();
  for (let p=0;p<periods;p++) {
    const x0=p*pw, xUp=x0+pw*duty;
    ctx.moveTo(x0,h*0.42); ctx.lineTo(x0,h*0.08); ctx.lineTo(xUp,h*0.08); ctx.lineTo(xUp,h*0.42); ctx.lineTo(x0+pw,h*0.42);
  }
  ctx.stroke();

  const phaseNorm = running ? (lastPhaseReg / P.TimerPeriod) : 0;
  ctx.strokeStyle = "#E85D4A"; ctx.lineWidth = 2; ctx.setLineDash([5,3]);
  ctx.beginPath();
  for (let p=0;p<periods;p++) {
    const x0=p*pw+pw*phaseNorm, xUp=x0+pw*duty;
    ctx.moveTo(Math.max(0,x0),h*0.9);
    if(x0>=0) ctx.lineTo(x0,h*0.55);
    ctx.lineTo(Math.min(w,xUp),h*0.55);
    if(xUp<=w) ctx.lineTo(xUp,h*0.9);
    if(x0+pw<=w) ctx.lineTo(x0+pw,h*0.9);
  }
  ctx.stroke(); ctx.setLineDash([]);
}

function renderCharts() {
  drawWaveform(dom.cvVoltIn,  hVoltIn,  "#2DD4A8", "rgba(45,212,168,0.15)");
  drawWaveform(dom.cvCurrIn,  hCurrIn,  "#F5A623", "rgba(245,166,35,0.15)");
  drawWaveform(dom.cvVoltOut, hVoltOut, "#2DD4A8", "rgba(45,212,168,0.15)");
  drawWaveform(dom.cvCurrOut, hCurrOut, "#F5A623", "rgba(245,166,35,0.15)");
  drawPWM(dom.cvPWM);
}

// ── Main loop ──────────────────────────────

function tick() {
  if (localSim) {
    for (let i = 0; i < 5; i++) localSimStep();
    renderCharts();
  }
}

// ── Init ───────────────────────────────────

function init() {
  cacheDom();
  updateActiveTag();

  bindParam("slI",  "inI",  "dI",  "CurrentSetPoint",  v => v.toFixed(1));
  bindParam("slV",  "inV",  "dV",  "VoltageSetPoint",  v => v.toFixed(0));
  bindParam("slKp", "inKp", "dKp", "ProportionalGain", v => v.toFixed(2));
  bindParam("slKi", "inKi", "dKi", "IntegralGain",     v => v.toFixed(1));

  dom.powerBtn.addEventListener("click", togglePower);
  dom.modeI.addEventListener("click", () => setMode(1));
  dom.modeV.addEventListener("click", () => setMode(0));
  dom.tuneBtn.addEventListener("click", toggleDrawer);
  dom.overlay.addEventListener("click", closeDrawer);
  document.getElementById("drawerClose").addEventListener("click", closeDrawer);
  document.addEventListener("keydown", e => { if (e.key === "Escape" && drawerOpen) closeDrawer(); });
  window.addEventListener("resize", renderCharts);

  connectWebSocket();
  setTimeout(() => { if (!wsConnected) enterLocalMode(); }, 1500);
  setInterval(tick, 50);
  tick();
}

document.addEventListener("DOMContentLoaded", init);
