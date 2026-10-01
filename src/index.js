// Postcard binary decoder for deserializing messages from the microcontroller
class PostcardDecoder {
  constructor(buffer) {
    this.view = new DataView(buffer);
    this.offset = 0;
  }

  // Decode variable-length integer (varint)
  // Uses continuation bit encoding: MSB is continuation flag, lower 7 bits are data
  readVarint() {
    let value = 0;
    let shift = 0;
    while (true) {
      const byte = this.view.getUint8(this.offset++);
      value |= (byte & 0x7F) << shift;
      if ((byte & 0x80) === 0) break;
      shift += 7;
    }
    return value;
  }

  // Decode 32-bit float (little-endian IEEE 754)
  readF32() {
    const value = this.view.getFloat32(this.offset, true); // true = little-endian
    this.offset += 4;
    return value;
  }

  // Decode Option<T> - 0x00 for None, 0x01 for Some(value)
  readOption(readFn) {
    const tag = this.view.getUint8(this.offset++);
    if (tag === 0x00) return null;
    return readFn.call(this);
  }

  // Decode UserEvent enum (varint tag: 0=Initializing, 1=Idle, 2=Stabilizing, 3=Grinding,
  // 4=WaitingForRemoval, 5=WaitingForCalibration, 6=Calibrating)
  readUserEvent() {
    const variant = this.readVarint();
    const states = ['Initializing', 'Idle', 'Stabilizing', 'Grinding', 'WaitingForRemoval',
      'WaitingForCalibration', 'Calibrating'];
    return states[variant] || 'Idle';
  }

  // Decode ScaleSetting struct
  readScaleSetting() {
    return {
      offset: this.readF32(),
      inv_variance: this.readF32(),
      factor: this.readF32()
    };
  }

  // Decode EtaReading struct (seconds until the grinder is expected to stop)
  readEta() {
    return { median: this.readF32(), lo: this.readF32(), hi: this.readF32() };
  }

  // Decode StopReason enum (varint tag: 0=Prediction, 1=RawWeight, 2=Timeout)
  readStopReason() {
    return ['Prediction', 'RawWeight', 'Timeout'][this.readVarint()] || 'Unknown';
  }

  // Decode WeightReading struct
  readWeightReading() {
    return {
      timestampMs: this.readVarint(),
      weight: this.readF32(),
      state: this.readUserEvent(),
      coffeeWeight: this.readOption(this.readF32),
      filteredWeight: this.readOption(this.readF32),
      eta: this.readOption(this.readEta)
    };
  }

  // Decode WsMessage enum (varint tag: 0=Connected, 1=StateChange, 2=Weight,
  // 3=TargetWeightChanged, 4=GrindFinished)
  readWsMessage() {
    const variant = this.readVarint();
    switch (variant) {
      case 0: // Connected
        return {
          type: 'connected',
          state: this.readUserEvent(),
          scaleSetting: this.readScaleSetting(),
          targetWeight: this.readF32(),
          timestampMs: this.readVarint(),
          leadTime: this.readF32()
        };
      case 1: // StateChange
        return {
          type: 'stateChange',
          state: this.readUserEvent(),
          scaleSetting: this.readScaleSetting(),
          targetWeight: this.readF32(),
          timestampMs: this.readVarint(),
          leadTime: this.readF32()
        };
      case 2: // Weight
        return {
          type: 'weight',
          reading: this.readWeightReading()
        };
      case 3: // TargetWeightChanged
        return {
          type: 'targetWeightChanged',
          targetWeight: this.readF32()
        };
      case 4: // GrindFinished
        return {
          type: 'grindFinished',
          stopReason: this.readStopReason(),
          stopWeight: this.readF32(),
          settledWeight: this.readOption(this.readF32),
          leadTimeObserved: this.readOption(this.readF32),
          leadTime: this.readF32()
        };
      default:
        throw new Error(`Unknown WsMessage variant: ${variant}`);
    }
  }
}

let lastStatus = '';
let currentWeight = 0;
let currentProgress = 0;
let targetWeight = null;
let ws = null;
let reconnectAttempts = 0;
const MAX_RECONNECT_DELAY = 30000; // 30 seconds

const statusMap = {
  'Initializing': 'initializing',
  'Idle': 'idle',
  'Stabilizing': 'stabilizing',
  'Grinding': 'grinding',
  'WaitingForRemoval': 'waiting',
  'WaitingForCalibration': 'calibration',
  'Calibrating': 'calibration'
};

const displayNames = {
  'WaitingForRemoval': 'Waiting for Removal',
  'WaitingForCalibration': 'Waiting for Calibration Weight',
  'Calibrating': 'Calibrating'
};

// Whether a calibration was started from this page, so we can guide the user
// through taring and weight removal which share states with normal operation.
let calibrationActive = false;

function storageGet(key) {
  try { return localStorage.getItem(key); } catch (e) { return null; }
}

function storageSet(key, value) {
  try { localStorage.setItem(key, value); } catch (e) { }
}

function calibrationWeight() {
  return parseFloat(document.getElementById('cal-weight').value);
}

function updateCalibrationUI(status) {
  const inCalibration = status === 'WaitingForCalibration' || status === 'Calibrating';
  if (inCalibration) calibrationActive = true;
  if (status === 'Idle') calibrationActive = false;

  const weight = calibrationWeight();
  let instructions = '';
  if (calibrationActive) {
    switch (status) {
      case 'Initializing': instructions = 'Taring - keep the scale empty...'; break;
      case 'WaitingForCalibration': instructions = `Place the ${weight} g reference weight on the scale.`; break;
      case 'Calibrating': instructions = 'Measuring - keep the scale still...'; break;
      case 'WaitingForRemoval': instructions = 'Calibration saved. Remove the weight.'; break;
    }
  }
  document.getElementById('cal-instructions').textContent = instructions;
  document.getElementById('cal-start').disabled = status !== 'Idle' || !(weight > 0);
  document.getElementById('cal-cancel').hidden =
    !(inCalibration || (calibrationActive && status === 'Initializing'));
}

async function postCalibration(path, body) {
  try {
    const response = await fetch(path, {
      method: 'POST',
      headers: { 'Content-Type': 'application/x-www-form-urlencoded' },
      body,
    });
    const text = await response.text();
    if (!response.ok) {
      addLog(`Calibration request failed: ${text}`);
      return false;
    }
    return true;
  } catch (e) {
    addLog(`Calibration request failed: ${e}`);
    return false;
  }
}

async function startCalibration() {
  const weight = calibrationWeight();
  if (!(weight > 0)) return;
  storageSet('calibrationWeight', String(weight));
  if (await postCalibration('/calibrate', new URLSearchParams({ weight }).toString())) {
    calibrationActive = true;
    addLog(`Calibration started with ${weight} g`);
    updateCalibrationUI(lastStatus);
  }
}

async function cancelCalibration() {
  if (await postCalibration('/calibrate/cancel', '')) {
    calibrationActive = false;
    addLog('Calibration cancelled');
    updateCalibrationUI(lastStatus);
  }
}

function setTargetWeight(weight) {
  const changed = targetWeight !== weight;
  targetWeight = weight;
  document.getElementById('target-weight').textContent = weight.toFixed(1);
  if (changed) drawGrindChart();
}

const HOLDER_FIELDS = ['single-weight', 'single-target', 'double-weight', 'double-target'];

async function loadHolders() {
  try {
    const response = await fetch('/holders');
    if (!response.ok) return;
    const holders = await response.json();
    const values = {
      'single-weight': holders.single.weight.toFixed(0),
      'single-target': holders.single.target.toFixed(1),
      'double-weight': holders.double.weight.toFixed(0),
      'double-target': holders.double.target.toFixed(1),
    };
    for (const id of HOLDER_FIELDS) {
      // Don't overwrite what the user is currently typing.
      const input = document.getElementById(id);
      if (document.activeElement !== input) input.value = values[id];
    }
  } catch (e) {
    console.error('Loading holders failed', e);
  }
}

async function saveHolders(event) {
  event.preventDefault();
  const [singleWeight, singleTarget, doubleWeight, doubleTarget] =
    HOLDER_FIELDS.map(id => parseFloat(document.getElementById(id).value));
  const instructions = document.getElementById('holders-instructions');
  if (![singleWeight, doubleWeight].every(w => w >= 100 && w <= 2000)) {
    instructions.textContent = 'Holder weights must be between 100 and 2000 g.';
    return;
  }
  if (![singleTarget, doubleTarget].every(w => w >= 1 && w <= 100)) {
    instructions.textContent = 'Target weights must be between 1 and 100 g.';
    return;
  }
  if (singleWeight === doubleWeight) {
    instructions.textContent = 'The holders need different weights to be told apart.';
    return;
  }
  try {
    const response = await fetch('/holders', {
      method: 'POST',
      headers: { 'Content-Type': 'application/x-www-form-urlencoded' },
      body: new URLSearchParams({
        single_weight: singleWeight,
        single_target: singleTarget,
        double_weight: doubleWeight,
        double_target: doubleTarget,
      }).toString(),
    });
    const text = await response.text();
    if (!response.ok) {
      instructions.textContent = `Saving holders failed: ${text}`;
      return;
    }
    instructions.textContent = '';
    document.activeElement.blur();
    addLog(`Holders set: single ${singleWeight} g → ${singleTarget.toFixed(1)} g, ` +
      `double ${doubleWeight} g → ${doubleTarget.toFixed(1)} g`);
  } catch (e) {
    instructions.textContent = `Saving holders failed: ${e}`;
  }
}

async function loadWifiStatus() {
  try {
    const response = await fetch('/wifi');
    const status = await response.json();
    let text;
    switch (status.mode) {
      case 'client': text = `Connected to ${status.ssid}.`; break;
      case 'setup':
        text = status.ssid
          ? `Could not connect to ${status.ssid}. Running the "grindy" setup access point.`
          : 'No network configured. Running the "grindy" setup access point.';
        break;
      default: text = status.ssid ? `Connecting to ${status.ssid}...` : 'Connecting...';
    }
    document.getElementById('wifi-status').textContent = text;
    const ssidInput = document.getElementById('wifi-ssid');
    if (status.ssid && !ssidInput.value) ssidInput.value = status.ssid;
  } catch (e) {
    document.getElementById('wifi-status').textContent = 'WiFi status unavailable';
  }
}

async function saveWifi(event) {
  event.preventDefault();
  const ssid = document.getElementById('wifi-ssid').value;
  const password = document.getElementById('wifi-password').value;
  const instructions = document.getElementById('wifi-instructions');
  if (password.length > 0 && password.length < 8) {
    instructions.textContent = 'The password must be empty or at least 8 characters.';
    return;
  }
  try {
    const response = await fetch('/wifi', {
      method: 'POST',
      headers: { 'Content-Type': 'application/x-www-form-urlencoded' },
      body: new URLSearchParams({ ssid, password }).toString(),
    });
    const text = await response.text();
    if (!response.ok) {
      instructions.textContent = `Saving WiFi settings failed: ${text}`;
      return;
    }
    instructions.textContent = `Saved. Grindy is now connecting to ${ssid}. Join that network and ` +
      'open Grindy at its new address. If connecting fails, the "grindy" setup access point ' +
      'comes back at 192.168.25.1.';
    addLog(`WiFi settings saved for ${ssid}`);
  } catch (e) {
    instructions.textContent = `Saving WiFi settings failed: ${e}`;
  }
}

function addLog(msg) {
  const logs = document.getElementById('logs');
  const entry = document.createElement('div');
  entry.className = 'log-entry';
  const time = new Date().toLocaleTimeString();
  entry.textContent = `[${time}] ${msg}`;
  logs.insertBefore(entry, logs.firstChild);
  if (logs.children.length > 20) logs.removeChild(logs.lastChild);
}

function updateUI(status, weight = null, progress = null, scaleSetting = null) {
  const statusText = document.getElementById('status-text');
  const statusContainer = document.getElementById('status-container');

  // Update status
  const displayStatus = displayNames[status] || status;
  statusText.textContent = displayStatus;

  // Update CSS class
  Object.values(statusMap).forEach(c => statusContainer.classList.remove(c));
  statusContainer.classList.add(statusMap[status] || 'idle');

  if (scaleSetting !== null) {
    document.getElementById('scale-offset').textContent = scaleSetting.offset.toFixed(2);
    document.getElementById('scale-std').textContent = Math.sqrt(
      1.0 / scaleSetting.inv_variance * Math.pow(scaleSetting.factor, 2)).toFixed(2);
    document.getElementById('scale-factor').textContent = scaleSetting.factor.toFixed(4);
  }

  // Update weight if provided
  if (weight !== null) {
    currentWeight = weight;
    document.getElementById('weight').textContent = weight.toFixed(1);
  }

  // Update progress if provided
  if (progress !== null) {
    currentProgress = progress;
    document.getElementById('progress').textContent = progress + '%';
  }

  // Log status changes
  if (lastStatus !== status) {
    addLog(`Status changed to: ${displayStatus}`);
    lastStatus = status;
    updateCalibrationUI(status);
  }
}

const stopReasonNames = {
  Prediction: 'forecast',
  RawWeight: 'weight reading',
  Timeout: 'timeout',
  Unknown: 'unknown reason'
};

function setLeadTime(leadTime) {
  document.getElementById('lead-time').textContent = leadTime.toFixed(2);
}

function updateEta(eta) {
  const value = document.getElementById('eta');
  const range = document.getElementById('eta-range');
  if (eta === null) {
    value.textContent = '--';
    range.textContent = 'seconds';
    return;
  }
  value.textContent = eta.median.toFixed(1);
  range.textContent = `seconds (${eta.lo.toFixed(1)}–${eta.hi.toFixed(1)})`;
}

function grindFinishedSummary(msg) {
  const target = targetWeight || 18.0;
  const settled = msg.settledWeight === null
    ? 'weight did not settle'
    : `settled at ${msg.settledWeight.toFixed(2)} g (target ${target.toFixed(1)} g)`;
  const learned = msg.leadTimeObserved === null
    ? `lead time stays ${msg.leadTime.toFixed(2)} s`
    : `lead time ${msg.leadTimeObserved.toFixed(2)} s observed, now ${msg.leadTime.toFixed(2)} s`;
  return `Stopped by ${stopReasonNames[msg.stopReason]} at ${msg.stopWeight.toFixed(2)} g, ` +
    `${settled}, ${learned}`;
}

function handleGrindFinished(msg) {
  setLeadTime(msg.leadTime);
  addLog(grindFinishedSummary(msg));
  if (grindTrace) {
    grindTrace.finished = msg;
    drawGrindChart();
  }
}

// Coffee weight over time for the current (or last) grind, plotted in the Grind Progress card.
const CHART = { left: 40, right: 590, top: 10, bottom: 215 };
const CHART_MIN_POINT_INTERVAL_MS = 50;
// How long to keep recording after grinding stopped so the chart shows the settled weight.
const CHART_TAIL_MS = 3000;
let grindTrace = null;

function niceStep(range, maxTicks) {
  const raw = range / maxTicks;
  const magnitude = Math.pow(10, Math.floor(Math.log10(raw)));
  for (const m of [1, 2, 5, 10]) {
    if (m * magnitude >= raw) return m * magnitude;
  }
  return 10 * magnitude;
}

function svgElement(name, attrs, text) {
  const el = document.createElementNS('http://www.w3.org/2000/svg', name);
  for (const [k, v] of Object.entries(attrs)) el.setAttribute(k, v);
  if (text !== undefined) el.textContent = text;
  return el;
}

function recordGrindReading(reading) {
  const recording = reading.state === 'Grinding' || reading.state === 'WaitingForRemoval';
  if (!recording || reading.coffeeWeight === null) {
    if (grindTrace) grindTrace.active = false;
    return false;
  }
  if (reading.state === 'Grinding' && !(grindTrace && grindTrace.active)) {
    grindTrace = { startMs: reading.timestampMs, endMs: null, points: [], active: true, finished: null };
  }
  if (!grindTrace || !grindTrace.active) return false;

  if (reading.state === 'WaitingForRemoval') {
    if (grindTrace.endMs === null) grindTrace.endMs = reading.timestampMs;
    if (reading.timestampMs - grindTrace.endMs > CHART_TAIL_MS) {
      grindTrace.active = false;
      return false;
    }
  }
  const points = grindTrace.points;
  const last = points[points.length - 1];
  if (last && reading.timestampMs - last.ms < CHART_MIN_POINT_INTERVAL_MS) return false;
  points.push({ ms: reading.timestampMs, weight: reading.coffeeWeight });
  return true;
}

function drawGrindChart() {
  if (!grindTrace || grindTrace.points.length === 0) return;
  const points = grindTrace.points;
  const target = targetWeight || 18.0;
  const lastPoint = points[points.length - 1];
  const duration = (lastPoint.ms - grindTrace.startMs) / 1000;

  const maxWeight = Math.max(target * 1.1, ...points.map(p => p.weight));
  const minWeight = Math.min(0, ...points.map(p => p.weight));
  const yStep = niceStep(maxWeight - minWeight, 5);
  const yMin = Math.floor(minWeight / yStep) * yStep;
  const yMax = Math.ceil(maxWeight / yStep) * yStep;
  const xStep = niceStep(Math.max(duration, 5), 6);
  const xMax = Math.max(Math.ceil(duration / xStep) * xStep, xStep);

  const x = s => CHART.left + (s / xMax) * (CHART.right - CHART.left);
  const y = g => CHART.bottom - ((g - yMin) / (yMax - yMin)) * (CHART.bottom - CHART.top);

  const grid = document.getElementById('chart-grid');
  grid.replaceChildren();
  for (let g = yMin; g <= yMax + yStep / 2; g += yStep) {
    grid.appendChild(svgElement('line', { class: 'chart-grid', x1: CHART.left, x2: CHART.right, y1: y(g), y2: y(g) }));
    grid.appendChild(svgElement('text', { class: 'chart-label', x: CHART.left - 6, y: y(g) + 4, 'text-anchor': 'end' }, `${+g.toFixed(1)}`));
  }
  for (let s = 0; s <= xMax + xStep / 2; s += xStep) {
    grid.appendChild(svgElement('text', { class: 'chart-label', x: x(s), y: CHART.bottom + 17, 'text-anchor': 'middle' }, `${+s.toFixed(1)}s`));
  }

  const targetLine = document.getElementById('chart-target');
  targetLine.setAttribute('y1', y(target));
  targetLine.setAttribute('y2', y(target));
  targetLine.setAttribute('visibility', 'visible');
  const targetLabel = document.getElementById('chart-target-label');
  targetLabel.setAttribute('y', y(target) - 5);
  targetLabel.textContent = `target ${target.toFixed(1)} g`;
  targetLabel.setAttribute('visibility', 'visible');

  document.getElementById('chart-line').setAttribute('points', points
    .map(p => `${x((p.ms - grindTrace.startMs) / 1000).toFixed(1)},${y(p.weight).toFixed(1)}`)
    .join(' '));

  const summary = document.getElementById('chart-summary');
  if (grindTrace.endMs === null) {
    summary.textContent = `Grinding... ${lastPoint.weight.toFixed(1)} g after ${duration.toFixed(1)} s`;
  } else if (grindTrace.finished) {
    summary.textContent = grindFinishedSummary(grindTrace.finished);
  } else {
    const grindTime = (grindTrace.endMs - grindTrace.startMs) / 1000;
    summary.textContent = `Last grind: ${lastPoint.weight.toFixed(1)} g in ${grindTime.toFixed(1)} s`;
  }
}

function handleWeight(reading) {
  if (recordGrindReading(reading)) drawGrindChart();

  const TARGET_WEIGHT = targetWeight || 18.0;
  let progress = 0;
  let displayWeight = reading.weight;

  // Use coffee weight if available (during grinding)
  if (reading.coffeeWeight !== undefined && reading.coffeeWeight !== null) {
    displayWeight = reading.filteredWeight ?? reading.coffeeWeight;
    progress = Math.min(100, Math.round((displayWeight / TARGET_WEIGHT) * 100));
  } else if (reading.state === 'Grinding') {
    // Fallback: assume total weight includes ~100g portafilter
    const estimatedCoffeeWeight = Math.max(0, reading.weight - 100);
    displayWeight = estimatedCoffeeWeight;
    progress = Math.min(100, Math.round((estimatedCoffeeWeight / TARGET_WEIGHT) * 100));
  }

  updateEta(reading.eta);
  updateUI(reading.state, displayWeight, progress);
}

function handleMessage(arrayBuffer) {
  try {
    const decoder = new PostcardDecoder(arrayBuffer);
    const msg = decoder.readWsMessage();

    switch (msg.type) {
      case 'connected':
        addLog('Connected to Grindy');
        setTargetWeight(msg.targetWeight);
        loadHolders();
        setLeadTime(msg.leadTime);
        updateUI(msg.state, null, null, msg.scaleSetting);
        break;

      case 'stateChange':
        addLog(`State: ${msg.state}`);
        setTargetWeight(msg.targetWeight);
        setLeadTime(msg.leadTime);
        updateUI(msg.state, null, null, msg.scaleSetting);
        break;

      case 'targetWeightChanged':
        setTargetWeight(msg.targetWeight);
        loadHolders();
        break;

      case 'weight':
        handleWeight(msg.reading);
        break;

      case 'grindFinished':
        handleGrindFinished(msg);
        break;

      default:
        console.warn('Unknown message type:', msg.type);
    }
  } catch (e) {
    console.error('Failed to decode WebSocket message:', e);
  }
}

function getReconnectDelay() {
  // Exponential backoff: 1s, 2s, 4s, 8s, 16s, 30s (max)
  const delay = Math.min(1000 * Math.pow(2, reconnectAttempts), MAX_RECONNECT_DELAY);
  reconnectAttempts++;
  return delay;
}

function connectWebSocket() {
  const protocol = window.location.protocol === 'https:' ? 'wss:' : 'ws:';
  const wsUrl = `${protocol}//${window.location.host}/ws`;

  addLog('Connecting...');

  ws = new WebSocket(wsUrl, ["grindy"]);
  ws.binaryType = 'arraybuffer'; // Receive binary data as ArrayBuffer

  ws.onopen = () => {
    console.log('WebSocket connected');
    reconnectAttempts = 0; // Reset on successful connection
    addLog('WebSocket connected');
  };

  ws.onmessage = (event) => {
    // With binaryType='arraybuffer', event.data is an ArrayBuffer
    handleMessage(event.data);
  };

  ws.onerror = (error) => {
    console.error('WebSocket error:', error);
    addLog('Connection error');
  };

  ws.onclose = () => {
    console.log('WebSocket disconnected');
    addLog('Disconnected - reconnecting...');
    ws = null;

    // Attempt to reconnect with exponential backoff
    const delay = getReconnectDelay();
    setTimeout(connectWebSocket, delay);
  };
}

// Initialize calibration controls and WebSocket connection on page load
const savedCalibrationWeight = storageGet('calibrationWeight');
if (savedCalibrationWeight) document.getElementById('cal-weight').value = savedCalibrationWeight;
document.getElementById('cal-weight').addEventListener('input', () => updateCalibrationUI(lastStatus));
document.getElementById('cal-start').addEventListener('click', startCalibration);
document.getElementById('cal-cancel').addEventListener('click', cancelCalibration);
document.getElementById('holders-form').addEventListener('submit', saveHolders);
document.getElementById('wifi-form').addEventListener('submit', saveWifi);
loadWifiStatus();
connectWebSocket();
