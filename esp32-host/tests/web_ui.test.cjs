// Run: node tests/web_ui.test.cjs
// Executes the actual JavaScript embedded in web_server.c with a browser mock.
const assert = require('node:assert/strict');
const fs = require('node:fs');
const vm = require('node:vm');
const source = fs.readFileSync('src/web_server.c', 'utf8');
const block = source.split('static const char index_html[] =')[1].split('/* Caller holds')[0];
const html = block.split('\n').filter(line => line.startsWith('"'))
    .flatMap(line => [...line.matchAll(/"(?:[^"\\]|\\.)*"/g)])
    .map(match => JSON.parse(match[0])).join('');
const script = html.split('<script>')[1].split('</script>')[0];

const elements = new Map();
function element(id) {
    if (!elements.has(id)) elements.set(id, {
        value: id === 'linear' ? '0.20' : id === 'angular' ? '1.00' : '50',
        style: {}, disabled: false, innerText: '', events: {},
        classList: {toggle() {}},
        addEventListener(type, cb) { this.events[type] = cb; },
        setPointerCapture() {},
        getBoundingClientRect() { return {left: 0, top: 0, width: 150, height: 150}; },
    });
    return elements.get(id);
}
const sockets = [];
const intervals = [];
const windowEvents = {};
const documentEvents = {};
class WebSocket {
    static OPEN = 1;
    constructor() { this.readyState = 1; this.sent = []; sockets.push(this); }
    send(data) { this.sent.push(data); }
    close() { this.readyState = 3; this.onclose(); }
}
const context = vm.createContext({
    WebSocket, location: {host: 'robot'},
    document: {
        hidden: false,
        hasFocus: () => true,
        addEventListener(type, cb) { documentEvents[type] = cb; },
        getElementById: element,
        querySelectorAll() { return []; },
    },
    window: {addEventListener(type, cb) { windowEvents[type] = cb; }},
    setInterval(callback, ms) { intervals.push({callback, ms}); },
    setTimeout() {},
    clearTimeout() {},
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

ws.onopen();
assert.deepEqual(ws.sent, [], 'opening the page must not take control');
assert.equal(element('controlState').innerText, 'INATIVO');
const commandTimer = intervals.find(timer => timer.ms === 50);
assert.ok(commandTimer, 'command timer remains periodic');
commandTimer.callback();
assert.deepEqual(ws.sent, [], 'inactive clients do not send CMD');

assert.equal(key('keydown', 'KeyW'), true);
assert.deepEqual(ws.sent, ['TAKE_CONTROL']);
assert.equal(run('haveControl'), true);
assert.equal(run('moveY'), 1);
key('keydown', 'KeyW', true);
assert.equal(ws.sent.filter(msg => msg === 'TAKE_CONTROL').length, 1,
    'key repeat must not flood TAKE_CONTROL');
commandTimer.callback();
assert.equal(ws.sent.at(-1), 'CMD,0.000,0.200,0.000,0,0');

active(false);
assert.equal(run('haveControl'), false);
assert.equal(run('moveX+moveY+rotX+kickPending'), 0);
assert.equal(element('controlState').innerText, 'INATIVO');
const beforeInactive = ws.sent.length;
commandTimer.callback();
assert.equal(ws.sent.length, beforeInactive);

element('moveJoy').events.pointerdown({pointerId: 1, clientX: 120, clientY: 75});
assert.equal(ws.sent.at(-1), 'TAKE_CONTROL');
const pointerTakeovers = ws.sent.filter(msg => msg === 'TAKE_CONTROL').length;
element('moveJoy').events.pointermove({clientX: 115, clientY: 75});
assert.equal(ws.sent.filter(msg => msg === 'TAKE_CONTROL').length, pointerTakeovers,
    'pointermove must not send TAKE_CONTROL');
run('sendCommand()');
assert.match(ws.sent.at(-1), /^CMD,0\.\d{3},0\.000,0\.000,0,0$/);

active(false);
run('kick()');
assert.equal(ws.sent.at(-1), 'TAKE_CONTROL');
assert.equal(run('kickPending'), 50);
run('sendCommand();sendCommand()');
assert.match(ws.sent.at(-2), /,50,1$/);
assert.match(ws.sent.at(-1), /,0,1$/);

active(false);
const beforeStop = ws.sent.length;
run('emergencyStop()');
assert.deepEqual(ws.sent.slice(beforeStop), ['EMERGENCY_STOP']);
assert.equal(run('haveControl'), false);

// WASD and Q/E still combine, and a new interaction retakes control once.
key('keydown', 'KeyW'); key('keydown', 'KeyA'); key('keydown', 'KeyQ');
assert.ok(Math.abs(run('moveY') - Math.SQRT1_2) < 1e-9);
assert.ok(Math.abs(run('moveX') + Math.SQRT1_2) < 1e-9);
assert.equal(run('rotX'), 1);
assert.equal(ws.sent.filter(msg => msg === 'TAKE_CONTROL').slice(-1).length, 1);
key('keyup', 'KeyA'); key('keyup', 'KeyQ'); key('keyup', 'KeyW');

const beforeBlur = ws.sent.length;
windowEvents.blur();
assert.deepEqual(ws.sent.slice(beforeBlur), ['CMD,0,0,0,0,1', 'RELEASE_CONTROL']);
assert.equal(run('haveControl'), false);
assert.equal(run('pressedKeys.size'), 0);
commandTimer.callback();
assert.equal(ws.sent.length, beforeBlur + 2);

windowEvents.focus();
key('keydown', 'KeyD');
const beforeHidden = ws.sent.length;
context.document.hidden = true;
documentEvents.visibilitychange();
assert.deepEqual(ws.sent.slice(beforeHidden), ['CMD,0,0,0,0,1', 'RELEASE_CONTROL']);
assert.equal(key('keydown', 'KeyQ'), false);
context.document.hidden = false;
documentEvents.visibilitychange();

ws.onmessage({data: JSON.stringify({battery: 12.3, rpm: [1,2,3,4], comm: true,
    watchdog: true, fault: 2, sequence: 42})});
assert.equal(element('sequence').innerText, 42);
assert.equal(element('battery').innerText, '12.300');
assert.equal(element('fault').innerText, '0x02');

key('keydown', 'KeyW');
const beforePagehide = ws.sent.length;
windowEvents.pagehide();
assert.deepEqual(ws.sent.slice(beforePagehide, beforePagehide + 2),
    ['CMD,0,0,0,0,1', 'RELEASE_CONTROL']);
assert.equal(ws.readyState, 3);
assert.equal(element('connection').innerText, 'OFFLINE');

console.log('PASS: interaction takeover, active status, WASD/joystick/kick, global stop, blur/visibility/pagehide');
