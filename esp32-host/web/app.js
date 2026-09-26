'use strict';

const DEBUG = true;

const UI = {
  connection: document.getElementById('connection'),
  controlState: document.getElementById('controlState'),
  battery: document.getElementById('battery'),
  rpm1: document.getElementById('rpm1'),
  rpm2: document.getElementById('rpm2'),
  rpm3: document.getElementById('rpm3'),
  rpm4: document.getElementById('rpm4'),
  comm: document.getElementById('comm'),
  watchdog: document.getElementById('watchdog'),
  sequence: document.getElementById('sequence'),
  fault: document.getElementById('fault'),
  rssi: document.getElementById('rssi'),
  linear: document.getElementById('linear'),
  linearText: document.getElementById('linearText'),
  angular: document.getElementById('angular'),
  angularText: document.getElementById('angularText'),
  kickPower: document.getElementById('kickPower'),
  kickText: document.getElementById('kickText'),
  kickButton: document.getElementById('kickButton'),
  stopButton: document.getElementById('stopButton'),
  moveJoy: document.getElementById('moveJoy'),
  moveKnob: document.getElementById('moveKnob'),
  rotJoy: document.getElementById('rotJoy'),
  rotKnob: document.getElementById('rotKnob'),
};

const state = {
  connected: false,
  active: false,
  telemetry: {},
  keys: new Set(),
  pageFocused: document.hasFocus(),
  closing: false,
  motion: { joystickX: 0, joystickY: 0, rotation: 0, x: 0, y: 0, omega: 0 },
  kickPending: 0,
  emergency: false,
};

const controlKeys = new Set(['KeyW', 'KeyA', 'KeyS', 'KeyD', 'KeyQ', 'KeyE', 'Space']);
const joystickResets = [];
const activeJoystickPointers = new Map();
let socket = null;

function debugLog(...args) {
  if (DEBUG) console.log(...args);
}

function setConnection(connected) {
  state.connected = connected;
  UI.connection.innerText = connected ? 'ONLINE' : 'OFFLINE';
  UI.connection.className = connected ? 'online' : 'offline';
}

function setActive(active) {
  if (state.active && !active) resetControls();
  state.active = active;
  UI.controlState.innerText = active ? 'ATIVO' : 'INATIVO';
}

function canInteract() {
  // document.hasFocus() is unreliable on mobile browsers. Visibility and the
  // WebSocket state are the portable gates for touch and pointer interaction.
  return state.connected && !document.hidden;
}

function sendMessage(message) {
  if (!socket || socket.readyState !== WebSocket.OPEN) return false;
  debugLog('[TX]', message);
  socket.send(message);
  return true;
}

function connectWebSocket() {
  socket = new WebSocket(`ws://${location.host}/ws`);

  socket.onopen = () => {
    debugLog('[WS] conectado');
    setConnection(true);
    setActive(false);
  };

  socket.onclose = () => {
    debugLog('[WS] desconectado');
    setConnection(false);
    setActive(false);
    if (!state.closing) window.setTimeout(connectWebSocket, 1000);
  };

  socket.onerror = error => {
    console.error('[WS] erro', error);
    socket.close();
  };

  socket.onmessage = event => {
    console.log('[RX RAW]', event.data);
    let data;
    try {
      data = JSON.parse(event.data);
    } catch (error) {
      console.error('[RX] JSON inválido', event.data, error);
      return;
    }
    handleMessage(data);
  };
}

function handleMessage(data) {
  debugLog('[RX]', data);
  if (!data || typeof data !== 'object') {
    console.warn('[RX] payload inválido', data);
    return;
  }
  if (data.type === 'telemetry') {
    updateTelemetry(data);
    return;
  }
  if (data.type === 'active') {
    handleActive(data);
    return;
  }
  console.warn('[RX] mensagem desconhecida', data);
}

function handleActive(data) {
  setActive(data.active === true);
}

function updateTelemetry(data) {
  state.telemetry = {...state.telemetry, ...data};

  const battery = Number(data.battery);
  if (data.battery !== undefined && Number.isFinite(battery)) {
    UI.battery.innerText = `${battery.toFixed(3)} V`;
  } else {
    console.warn('[RX] telemetry sem battery valida', data.battery);
  }

  if (Array.isArray(data.rpm) && data.rpm.length >= 4) {
    UI.rpm1.innerText = String(data.rpm[0]);
    UI.rpm2.innerText = String(data.rpm[1]);
    UI.rpm3.innerText = String(data.rpm[2]);
    UI.rpm4.innerText = String(data.rpm[3]);
  } else {
    console.warn('[RX] telemetry sem rpm[4]');
  }

  if (data.comm !== undefined) UI.comm.innerText = data.comm ? 'OK' : 'LOST';
  if (data.watchdog !== undefined) UI.watchdog.innerText = data.watchdog ? 'OK' : 'ERRO';
  if (data.sequence !== undefined) UI.sequence.innerText = String(data.sequence);

  const fault = Number(data.fault);
  if (data.fault !== undefined && Number.isFinite(fault)) {
    UI.fault.innerText = `0x${fault.toString(16).padStart(2, '0').toUpperCase()}`;
  }

  const rssi = Number(data.rssi);
  if (data.rssi !== undefined && Number.isFinite(rssi)) {
    UI.rssi.innerText = `${rssi} dBm`;
  }
}

function updateMotion() {
  const key = code => state.keys.has(code) ? 1 : 0;
  const x = state.motion.joystickX + key('KeyD') - key('KeyA');
  const y = state.motion.joystickY + key('KeyW') - key('KeyS');
  const scale = Math.max(1, Math.hypot(x, y));
  state.motion.x = x / scale;
  state.motion.y = y / scale;
  state.motion.omega = Math.max(-1, Math.min(1,
    state.motion.rotation + key('KeyQ') - key('KeyE')));
  if (state.motion.x || state.motion.y || state.motion.omega) state.emergency = false;
}

function resetControls() {
  state.keys.clear();
  Object.assign(state.motion, {joystickX: 0, joystickY: 0, rotation: 0, x: 0, y: 0, omega: 0});
  state.kickPending = 0;
  state.emergency = true;
  joystickResets.forEach(reset => reset());
}

function requestControl() {
  if (!canInteract()) return false;
  if (!state.active) {
    if (!sendMessage('TAKE_CONTROL')) return false;
    setActive(true);
  }
  return true;
}

function releaseControl() {
  if (state.active) sendMessage('RELEASE_CONTROL');
  setActive(false);
}

function sendCommand() {
  if (!state.active || !canInteract()) return;
  const vx = state.motion.x * Number(UI.linear.value);
  const vy = state.motion.y * Number(UI.linear.value);
  const omega = state.motion.omega * Number(UI.angular.value);
  const stationary = Math.abs(vx) < 0.001 && Math.abs(vy) < 0.001 && Math.abs(omega) < 0.001;
  const brake = state.emergency || stationary ? 1 : 0;
  sendMessage(`CMD,${vx.toFixed(3)},${vy.toFixed(3)},${omega.toFixed(3)},${state.kickPending},${brake}`);
  state.kickPending = 0;
}

function sendSafeStop() {
  if (state.active) sendMessage('CMD,0,0,0,0,1');
}

function kick() {
  if (!requestControl()) return;
  state.kickPending = Number.parseInt(UI.kickPower.value, 10);
}

function emergencyStop() {
  if (!canInteract()) return;
  sendMessage('EMERGENCY_STOP');
  resetControls();
  setActive(false);
}

function pausePage() {
  state.pageFocused = false;
  resetControls();
  sendSafeStop();
  releaseControl();
}

function setupJoystick(base, knob, callback, rotationOnly) {
  let activePointerId = null;

  function update(event) {
    if (!state.active || event.pointerId !== activePointerId) return;
    event.preventDefault?.();
    const rect = base.getBoundingClientRect();
    let dx = event.clientX - (rect.left + rect.width / 2);
    let dy = event.clientY - (rect.top + rect.height / 2);
    if (rotationOnly) dy = 0;
    const radius = Math.max(1, Math.min(rect.width, rect.height) / 2 - 30);
    const length = Math.hypot(dx, dy);
    if (length > radius) {
      dx = dx / length * radius;
      dy = dy / length * radius;
    }
    console.log('[JOYSTICK MOVE]', dx, dy);
    knob.style.transform = `translate(${dx}px,${dy}px)`;
    callback(dx / radius, -dy / radius);
  }

  function stop(event) {
    if (event && event.pointerId !== activePointerId) return;
    const pointerId = activePointerId;
    activePointerId = null;
    if (pointerId !== null) activeJoystickPointers.delete(pointerId);
    knob.style.transform = 'translate(0px,0px)';
    callback(0, 0);
  }

  joystickResets.push(stop);
  base.addEventListener('pointerdown', event => {
    if (activePointerId !== null) return;
    event.preventDefault?.();
    state.pageFocused = true;
    if (!requestControl()) return;
    activePointerId = event.pointerId;
    activeJoystickPointers.set(event.pointerId, stop);
    base.setPointerCapture(event.pointerId);
    console.log('[JOYSTICK DOWN]', event.clientX, event.clientY);
    update(event);
  });
  base.addEventListener('pointermove', update);
  base.addEventListener('lostpointercapture', stop);
  base.addEventListener('pointerup', stop);
  base.addEventListener('pointercancel', stop);
}

function stopJoystickPointer(event) {
  activeJoystickPointers.get(event.pointerId)?.(event);
}

window.addEventListener('pointerup', stopJoystickPointer);
window.addEventListener('pointercancel', stopJoystickPointer);

UI.linear.addEventListener('input', () => { UI.linearText.innerText = Number(UI.linear.value).toFixed(2); });
UI.angular.addEventListener('input', () => { UI.angularText.innerText = Number(UI.angular.value).toFixed(2); });
UI.kickPower.addEventListener('input', () => { UI.kickText.innerText = UI.kickPower.value; });
UI.kickButton.addEventListener('click', kick);
UI.stopButton.addEventListener('click', emergencyStop);

setupJoystick(UI.moveJoy, UI.moveKnob, (x, y) => {
  state.motion.joystickX = x;
  state.motion.joystickY = y;
  updateMotion();
}, false);
setupJoystick(UI.rotJoy, UI.rotKnob, x => {
  state.motion.rotation = x;
  updateMotion();
}, true);

window.addEventListener('keydown', event => {
  if (!controlKeys.has(event.code) || !canInteract()) return;
  event.preventDefault();
  if (event.repeat) return;
  if (event.code === 'Space') {
    emergencyStop();
    state.keys.add('Space');
    return;
  }
  if (state.keys.has('Space') || !requestControl()) return;
  state.keys.add(event.code);
  updateMotion();
});

window.addEventListener('keyup', event => {
  if (!controlKeys.has(event.code)) return;
  event.preventDefault();
  state.keys.delete(event.code);
  updateMotion();
});

window.addEventListener('blur', pausePage);
window.addEventListener('focus', () => { state.pageFocused = true; });
document.addEventListener('visibilitychange', () => {
  if (document.hidden) pausePage();
  else state.pageFocused = document.hasFocus();
});
window.addEventListener('pagehide', () => {
  state.closing = true;
  resetControls();
  sendSafeStop();
  releaseControl();
  if (socket && socket.readyState === WebSocket.OPEN) socket.close();
});

window.setInterval(sendCommand, 50);
setConnection(false);
setActive(false);
connectWebSocket();
