// Run: node tests/web_ui.test.cjs
// Executes web/app.js with a browser mock and validates the real web assets.
const assert = require('node:assert/strict');
const fs = require('node:fs');
const vm = require('node:vm');

const html = fs.readFileSync('web/index.html', 'utf8');
const script = fs.readFileSync('web/app.js', 'utf8');
const style = fs.readFileSync('web/style.css', 'utf8');
const elements = new Map();

function element(id) {
    if (!elements.has(id)) elements.set(id, {
        value: id === 'linear' ? '0.20' : id === 'angular' ? '1.00' : '50',
        style: {}, disabled: false, innerText: '', className: '', events: {}, capturedPointer: null,
        addEventListener(type, cb) { this.events[type] = cb; },
        setPointerCapture(pointerId) { this.capturedPointer = pointerId; },
        getBoundingClientRect() { return {left: 0, top: 0, width: 150, height: 150}; },
    });
    return elements.get(id);
}

const sockets = [];
const intervals = [];
const timeouts = [];
const windowEvents = {};
const documentEvents = {};
const consoleMessages = [];

class WebSocket {
    static OPEN = 1;
    constructor() { this.readyState = 1; this.sent = []; sockets.push(this); }
    send(data) { this.sent.push(data); }
    close() { this.readyState = 3; this.onclose(); }
}

const context = vm.createContext({
    WebSocket,
    location: {host: 'robot'},
    console: {
        log(...args) { consoleMessages.push(['log', ...args]); },
        error(...args) { consoleMessages.push(['error', ...args]); },
        warn(...args) { consoleMessages.push(['warn', ...args]); },
    },
    document: {
        hidden: false,
        hasFocus: () => false,
        addEventListener(type, cb) { documentEvents[type] = cb; },
        getElementById: element,
    },
    window: {
        addEventListener(type, cb) { windowEvents[type] = cb; },
        setInterval(callback, ms) { intervals.push({callback, ms}); },
        setTimeout(callback, ms) { timeouts.push({callback, ms}); },
    },
});

vm.runInContext(script, context);
const run = code => vm.runInContext(code, context);
const ws = sockets[0];
const active = value => ws.onmessage({data: JSON.stringify({type: 'active', active: value})});
function key(type, code, repeat = false) {
    let prevented = false;
    windowEvents[type]({code, repeat, preventDefault() { prevented = true; }});
    return prevented;
}

for (const id of ['battery','rpm1','rpm2','rpm3','rpm4','watchdog','sequence','fault','rssi']) {
    assert.match(html, new RegExp(`id="${id}"`));
}
assert.match(html, /href="\/style\.css"/);
assert.match(html, /src="\/app\.js"/);
assert.match(style, /\.telemetry/);
assert.match(style, /\.joystick[\s\S]*touch-action:\s*none/);
assert.match(style, /\.joystick[\s\S]*user-select:\s*none/);

ws.onopen();
assert.deepEqual(ws.sent, [], 'opening the page must not take control');
assert.equal(run('state.connected'), true);
assert.equal(run('state.active'), false);
assert.equal(element('connection').innerText, 'ONLINE');
const commandTimer = intervals.find(timer => timer.ms === 50);
assert.ok(commandTimer, 'command timer remains periodic');
commandTimer.callback();
assert.deepEqual(ws.sent, [], 'inactive clients do not send CMD');

assert.equal(key('keydown', 'KeyW'), true);
assert.deepEqual(ws.sent, ['TAKE_CONTROL']);
assert.equal(run('state.active'), true);
assert.equal(run('state.motion.y'), 1);
key('keydown', 'KeyW', true);
assert.equal(ws.sent.filter(msg => msg === 'TAKE_CONTROL').length, 1);
commandTimer.callback();
assert.equal(ws.sent.at(-1), 'CMD,0.000,0.200,0.000,0,0');

active(false);
assert.equal(run('state.active'), false);
assert.equal(run('state.motion.x+state.motion.y+state.motion.omega+state.kickPending'), 0);
const beforeInactive = ws.sent.length;
commandTimer.callback();
assert.equal(ws.sent.length, beforeInactive);

let pointerPrevented = false;
element('moveJoy').events.pointerdown({pointerId: 1, clientX: 120, clientY: 75,
    preventDefault() { pointerPrevented = true; }});
assert.equal(pointerPrevented, true);
assert.equal(element('moveJoy').capturedPointer, 1);
assert.equal(ws.sent.at(-1), 'TAKE_CONTROL');
const pointerTakeovers = ws.sent.filter(msg => msg === 'TAKE_CONTROL').length;
element('moveJoy').events.pointermove({pointerId: 1, clientX: 115, clientY: 75,
    preventDefault() {}});
assert.equal(ws.sent.filter(msg => msg === 'TAKE_CONTROL').length, pointerTakeovers);
run('sendCommand()');
assert.match(ws.sent.at(-1), /^CMD,0\.\d{3},0\.000,0\.000,0,0$/);
assert.ok(consoleMessages.some(entry => entry[0] === 'log' && entry[1] === '[JOYSTICK DOWN]'));
assert.ok(consoleMessages.some(entry => entry[0] === 'log' && entry[1] === '[JOYSTICK MOVE]'));
windowEvents.pointerup({pointerId: 1});
assert.equal(run('state.motion.x'), 0);

active(false);
element('kickButton').events.click();
assert.equal(ws.sent.at(-1), 'TAKE_CONTROL');
assert.equal(run('state.kickPending'), 50);
run('sendCommand();sendCommand()');
assert.match(ws.sent.at(-2), /,50,1$/);
assert.match(ws.sent.at(-1), /,0,1$/);

active(false);
const beforeStop = ws.sent.length;
element('stopButton').events.click();
assert.deepEqual(ws.sent.slice(beforeStop), ['EMERGENCY_STOP']);
assert.equal(run('state.active'), false);

key('keydown', 'KeyW'); key('keydown', 'KeyA'); key('keydown', 'KeyQ');
assert.ok(Math.abs(run('state.motion.y') - Math.SQRT1_2) < 1e-9);
assert.ok(Math.abs(run('state.motion.x') + Math.SQRT1_2) < 1e-9);
assert.equal(run('state.motion.omega'), 1);
key('keyup', 'KeyA'); key('keyup', 'KeyQ'); key('keyup', 'KeyW');

const beforeBlur = ws.sent.length;
windowEvents.blur();
assert.deepEqual(ws.sent.slice(beforeBlur), ['CMD,0,0,0,0,1', 'RELEASE_CONTROL']);
assert.equal(run('state.active'), false);
assert.equal(run('state.keys.size'), 0);

windowEvents.focus();
key('keydown', 'KeyD');
const beforeHidden = ws.sent.length;
context.document.hidden = true;
documentEvents.visibilitychange();
assert.deepEqual(ws.sent.slice(beforeHidden), ['CMD,0,0,0,0,1', 'RELEASE_CONTROL']);
context.document.hidden = false;
documentEvents.visibilitychange();

assert.equal(run('state.active'), false);
ws.onmessage({data: JSON.stringify({type: 'telemetry', battery: 11.835,
    rpm: [0,0,0,0], comm: true, watchdog: true, fault: 0, sequence: 10,
    rssi: -60})});
assert.equal(element('battery').innerText, '11.835 V');
assert.deepEqual([element('rpm1').innerText, element('rpm2').innerText,
    element('rpm3').innerText, element('rpm4').innerText], ['0','0','0','0']);
assert.equal(element('watchdog').innerText, 'OK');
assert.equal(element('sequence').innerText, '10');
assert.equal(element('fault').innerText, '0x00');
assert.equal(element('rssi').innerText, '-60 dBm');
assert.equal(run('state.active'), false, 'telemetry must not take control');
assert.equal(run('state.telemetry.battery'), 11.835);
assert.ok(consoleMessages.some(entry => entry[0] === 'log' && entry[1] === '[RX]'));
assert.ok(consoleMessages.some(entry => entry[0] === 'log' && entry[1] === '[RX RAW]'));

ws.onmessage({data: JSON.stringify({type: 'telemetry', battery: 11.799,
    rpm: [0,0,0,0], rssi: -37})});
assert.equal(element('battery').innerText, '11.799 V');
assert.equal(element('rpm4').innerText, '0');
assert.equal(element('rssi').innerText, '-37 dBm');

const renderedBattery = element('battery').innerText;
ws.onmessage({data: JSON.stringify({type: 'telemetry'})});
ws.onmessage({data: '{invalid'});
ws.onmessage({data: JSON.stringify({type: 'future_api_message'})});
assert.equal(element('battery').innerText, renderedBattery);
assert.ok(consoleMessages.some(entry => entry[0] === 'warn'));
assert.ok(consoleMessages.some(entry => entry[0] === 'error'));

key('keydown', 'KeyW');
const beforePagehide = ws.sent.length;
windowEvents.pagehide();
assert.deepEqual(ws.sent.slice(beforePagehide, beforePagehide + 2),
    ['CMD,0,0,0,0,1', 'RELEASE_CONTROL']);
assert.equal(ws.readyState, 3);
assert.equal(element('connection').innerText, 'OFFLINE');
assert.equal(timeouts.length, 0, 'intentional close must not reconnect');

console.log('PASS: modular frontend, message router, telemetry, controls, safety and reconnect state');
