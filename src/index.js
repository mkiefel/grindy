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

  // Decode UserEvent enum (varint tag: 0=Initializing, 1=Idle, 2=Stabilizing, 3=Grinding, 4=WaitingForRemoval)
  readUserEvent() {
    const variant = this.readVarint();
    const states = ['Initializing', 'Idle', 'Stabilizing', 'Grinding', 'WaitingForRemoval'];
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

  // Decode WsMessage enum (varint tag: 0=Connected, 1=StateChange, 2=WeightBatch)
  readWsMessage() {
    const variant = this.readVarint();
    switch (variant) {
      case 0: // Connected
        return {
          type: 'connected',
          state: this.readUserEvent(),
          scaleSetting: this.readScaleSetting(),
          timestampMs: this.readVarint(),
        };
      case 1: // StateChange
        return {
          type: 'stateChange',
          state: this.readUserEvent(),
          timestampMs: this.readVarint()
        };
      case 2: // WeightBatch
        return {
          type: 'weightBatch',
          readings: this.readVec(this.readWeightReading)
        };
      default:
        throw new Error(`Unknown WsMessage variant: ${variant}`);
    }
  }
}

let lastStatus = '';
let currentWeight = 0;
let currentProgress = 0;
let ws = null;
let reconnectAttempts = 0;
const MAX_RECONNECT_DELAY = 30000; // 30 seconds

const statusMap = {
  'Initializing': 'initializing',
  'Idle': 'idle',
  'Stabilizing': 'stabilizing',
  'Grinding': 'grinding',
  'WaitingForRemoval': 'waiting'
};

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
  const displayStatus = status === 'WaitingForRemoval' ? 'Waiting for Removal' : status;
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
  }
}

function handleWeightBatch(readings) {
  if (readings && readings.length > 0) {
    // Use the most recent reading
    const latest = readings[readings.length - 1];

    // Calculate progress if grinding (assuming 18g target)
    const TARGET_WEIGHT = 18.0;
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
        updateUI(msg.state, null, null, msg.scaleSetting);
        break;

      case 'stateChange':
        addLog(`State: ${msg.state}`);
        updateUI(msg.state);
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

// Initialize WebSocket connection on page load
connectWebSocket();
