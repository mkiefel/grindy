let lastStatus = '';
let currentWeight = 0;
let currentProgress = 0;
let ws = null;
let reconnectAttempts = 0;
const MAX_RECONNECT_DELAY = 30000; // 30 seconds

const statusMap = {
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

function updateUI(status, weight = null, progress = null) {
  const statusText = document.getElementById('status-text');
  const statusContainer = document.getElementById('status-container');
  const statusMsg = document.getElementById('status-msg');

  // Update status
  const displayStatus = status === 'WaitingForRemoval' ? 'Waiting for Removal' : status;
  statusText.textContent = displayStatus;

  // Update CSS class
  Object.values(statusMap).forEach(c => statusContainer.classList.remove(c));
  statusContainer.classList.add(statusMap[status] || 'idle');

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

function handleMessage(data) {
  try {
    const msg = JSON.parse(data);

    switch (msg.type) {
      case 'connected':
        addLog('Connected to Grindy');
        updateUI(msg.state);
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
    console.error('Failed to parse WebSocket message:', e, data);
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

  ws = new WebSocket(wsUrl);

  ws.onopen = () => {
    console.log('WebSocket connected');
    reconnectAttempts = 0; // Reset on successful connection
    addLog('WebSocket connected');
  };

  ws.onmessage = (event) => {
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
