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

  // Decode WeightReading struct
  readWeightReading() {
    return {
      timestampMs: this.readVarint(),
      weight: this.readF32(),
      state: this.readUserEvent(),
      coffeeWeight: this.readOption(this.readF32)
    };
  }

  // Decode Vec<T> - varint length followed by elements
  readVec(readFn) {
    const length = this.readVarint();
    const vec = [];
    for (let i = 0; i < length; i++) {
      vec.push(readFn.call(this));
    }
    return vec;
  }

  // Decode WsMessage enum (varint tag: 0=Connected, 1=StateChange, 2=WeightBatch,
  // 3=TargetWeightChanged)
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
        };
      case 1: // StateChange
        return {
          type: 'stateChange',
          state: this.readUserEvent(),
          scaleSetting: this.readScaleSetting(),
          targetWeight: this.readF32(),
          timestampMs: this.readVarint()
        };
      case 2: // WeightBatch
        return {
          type: 'weightBatch',
          readings: this.readVec(this.readWeightReading)
        };
      case 3: // TargetWeightChanged
        return {
          type: 'targetWeightChanged',
          targetWeight: this.readF32()
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
  // Don't overwrite what the user is currently typing.
  const input = document.getElementById('target-input');
  if (document.activeElement !== input && (changed || !input.value)) {
    input.value = weight.toFixed(1);
  }
}

async function saveTargetWeight(event) {
  event.preventDefault();
  const weight = parseFloat(document.getElementById('target-input').value);
  const instructions = document.getElementById('target-instructions');
  if (!(weight >= 1 && weight <= 100)) {
    instructions.textContent = 'The target weight must be between 1 and 100 g.';
    return;
  }
  try {
    const response = await fetch('/target-weight', {
      method: 'POST',
      headers: { 'Content-Type': 'application/x-www-form-urlencoded' },
      body: new URLSearchParams({ weight }).toString(),
    });
    const text = await response.text();
    if (!response.ok) {
      instructions.textContent = `Saving target weight failed: ${text}`;
      return;
    }
    instructions.textContent = '';
    document.getElementById('target-input').blur();
    setTargetWeight(weight);
    addLog(`Target weight set to ${weight.toFixed(1)} g`);
  } catch (e) {
    instructions.textContent = `Saving target weight failed: ${e}`;
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

function handleWeightBatch(readings) {
  if (readings && readings.length > 0) {
    // Use the most recent reading
    const latest = readings[readings.length - 1];

    const TARGET_WEIGHT = targetWeight || 18.0;
    let progress = 0;
    let displayWeight = latest.weight;

    // Use coffee weight if available (during grinding)
    if (latest.coffeeWeight !== undefined && latest.coffeeWeight !== null) {
      displayWeight = latest.coffeeWeight;
      progress = Math.min(100, Math.round((latest.coffeeWeight / TARGET_WEIGHT) * 100));
    } else if (latest.state === 'Grinding') {
      // Fallback: assume total weight includes ~100g portafilter
      const estimatedCoffeeWeight = Math.max(0, latest.weight - 100);
      displayWeight = estimatedCoffeeWeight;
      progress = Math.min(100, Math.round((estimatedCoffeeWeight / TARGET_WEIGHT) * 100));
    }

    updateUI(latest.state, displayWeight, progress);
  }
}

function handleMessage(arrayBuffer) {
  try {
    const decoder = new PostcardDecoder(arrayBuffer);
    const msg = decoder.readWsMessage();

    switch (msg.type) {
      case 'connected':
        addLog('Connected to Grindy');
        setTargetWeight(msg.targetWeight);
        updateUI(msg.state, null, null, msg.scaleSetting);
        break;

      case 'stateChange':
        addLog(`State: ${msg.state}`);
        setTargetWeight(msg.targetWeight);
        updateUI(msg.state, null, null, msg.scaleSetting);
        break;

      case 'targetWeightChanged':
        setTargetWeight(msg.targetWeight);
        break;

      case 'weightBatch':
        handleWeightBatch(msg.readings);
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
document.getElementById('target-form').addEventListener('submit', saveTargetWeight);
document.getElementById('wifi-form').addEventListener('submit', saveWifi);
loadWifiStatus();
connectWebSocket();
