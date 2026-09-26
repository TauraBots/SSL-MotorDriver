// Run: node tests/web_ui.test.cjs
// Executes the actual JavaScript embedded in web_server.c with a browser mock.
const assert = require('node:assert/strict');
const fs = require('node:fs');
const vm = require('node:vm');
const source = fs.readFileSync('src/web_server.c', 'utf8');
const block = source.split('static const char index_html[] =')[1].split('/* Caller holds')[0];
const config = fs.readFileSync('include/app_config.h', 'utf8');
const heartbeatMs = Number(config.match(/#define CONTROLLER_HEARTBEAT_MS (\d+)/)[1]);
const expanded = block.replace('STRINGIFY_VALUE(CONTROLLER_HEARTBEAT_MS)', JSON.stringify(String(heartbeatMs)));
const html = expanded.split('\n').filter(line => line.startsWith('"'))
    .flatMap(line => [...line.matchAll(/"(?:[^"\\]|\\.)*"/g)])
    .map(m => JSON.parse(m[0])).join('');
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
        querySelectorAll(selector) {
            return (selector === '.joystick' ? ['moveJoy', 'rotJoy'] :
                ['linear', 'angular', 'kickPower', 'kick', 'stop']).map(element);
        },
    },
    window: {addEventListener(type, cb) { windowEvents[type] = cb; }},
    setInterval(callback, ms) { intervals.push({callback, ms}); }, setTimeout() {},
});
vm.runInContext(script, context);
const run = code => vm.runInContext(code, context);
const ws = sockets[0];
const status = controller => ws.onmessage({data: JSON.stringify({type: 'control', controller})});
function key(type, code, repeat = false) {
    let prevented = false;
    windowEvents[type]({code, repeat, preventDefault() { prevented = true; }});
    return prevented;
}
ws.onopen();
assert.deepEqual(ws.sent, ['CLAIM_CONTROL'], 'exactly one automatic claim on open');
ws.sent.length = 0;
const heartbeat = intervals.find(timer => timer.ms === heartbeatMs);
assert.ok(heartbeat, 'heartbeat uses configured interval');
heartbeat.callback();
run('sendCommand();kick();emergencyStop();');
assert.equal(ws.sent.length, 0, 'spectators must not send commands');
assert.equal(element('kick').disabled, true);
run('toggleControl()');
assert.equal(ws.sent.pop(), 'CLAIM_CONTROL');
status(true);
assert.equal(element('role').innerText, 'CONTROLADOR');
assert.equal(element('kick').disabled, false);
heartbeat.callback();
assert.equal(ws.sent.pop(), 'HEARTBEAT');
status(true);
assert.equal(ws.sent.filter(msg => msg === 'CLAIM_CONTROL').length, 0,
    'control status must not trigger additional automatic claims');
element('moveJoy').events.pointerdown({pointerId: 1, clientX: 120, clientY: 75});
run('kick();sendCommand();sendCommand();');
assert.match(ws.sent.at(-2), /^CMD,0.200,0.000,0.000,50,0$/);
assert.match(ws.sent.at(-1), /,0,0$/);
status(false);
const before = ws.sent.length;
element('moveJoy').events.pointermove({clientX: 120, clientY: 75});
run('sendCommand();kick();');
heartbeat.callback();
assert.equal(ws.sent.length, before);
assert.equal(run('moveX+moveY+rotX+kickPending'), 0);
assert.equal(element('moveKnob').style.transform, 'translate(0px,0px)');
status(true);
run('sendCommand()');
assert.equal(ws.sent.at(-1), 'CMD,0.000,0.000,0.000,0,1');
run('toggleControl()');
assert.equal(ws.sent.at(-1), 'RELEASE_CONTROL');
assert.equal(run('isController'), false);
ws.onmessage({data: JSON.stringify({battery: 12.3, rpm: [1,2,3,4], comm: true,
    watchdog: true, fault: 2, sequence: 42})});
assert.equal(element('sequence').innerText, 42);
assert.equal(element('battery').innerText, '12.300');
assert.equal(element('fault').innerText, '0x02');

// Spectators cannot change keyboard state or queue a movement command.
assert.equal(key('keydown', 'KeyW'), false);
assert.equal(key('keyup', 'KeyW'), false);
assert.equal(run('pressedKeys.size'), 0);
status(true);
assert.equal(key('keydown', 'KeyW'), true);
assert.equal(run('moveY'), 1);
assert.equal(run('moveX'), 0);
key('keydown', 'KeyA');
assert.ok(Math.abs(run('moveY') - Math.SQRT1_2) < 1e-9);
assert.ok(Math.abs(run('moveX') + Math.SQRT1_2) < 1e-9);
key('keyup', 'KeyA');
key('keydown', 'KeyQ');
assert.equal(run('moveY'), 1);
assert.equal(run('rotX'), 1);
key('keydown', 'KeyE');
assert.equal(run('rotX'), 0);
key('keyup', 'KeyQ');
assert.equal(run('rotX'), -1);
key('keyup', 'KeyE');
key('keydown', 'KeyS');
assert.equal(run('moveY'), 0);
key('keyup', 'KeyW');
assert.equal(run('moveY'), -1);
key('keyup', 'KeyS');
key('keydown', 'KeyD');
assert.equal(run('moveX'), 1);
key('keyup', 'KeyD');
assert.equal(run('moveX'), 0);

// Movement keys never touch kickPending or create an extra send path.
const keyboardStart = ws.sent.length;
run('kick()');
key('keydown', 'KeyW');
key('keydown', 'KeyW', true);
key('keyup', 'KeyW');
assert.equal(run('kickPending'), 50);
assert.equal(ws.sent.length, keyboardStart);
run('sendCommand();sendCommand()');
assert.match(ws.sent.at(-2), /,50,1$/);
assert.match(ws.sent.at(-1), /,0,1$/);

// Releasing touch must not erase a key still held, and vice versa.
element('moveJoy').events.pointerdown({pointerId: 2, clientX: 120, clientY: 75});
key('keydown', 'KeyW');
element('moveJoy').events.pointerup();
assert.equal(run('moveY'), 1);
assert.equal(run('moveX'), 0);
key('keyup', 'KeyW');
assert.equal(run('moveY'), 0);

key('keydown', 'KeyW');
assert.equal(key('keydown', 'Space'), true);
assert.equal(run('moveX+moveY+rotX+kickPending'), 0);
assert.equal(run('emergency'), true);
key('keydown', 'KeyW', true);
key('keydown', 'KeyD');
assert.equal(run('moveX+moveY'), 0);
key('keyup', 'Space');
key('keydown', 'KeyW', true);
assert.equal(run('moveY'), 0, 'held keys must not restart after STOP');
key('keyup', 'KeyW');
key('keydown', 'KeyW');
windowEvents.blur();
assert.equal(run('pressedKeys.size'), 0);
assert.equal(ws.sent.at(-1), 'CMD,0.000,0.000,0.000,0,1');
assert.equal(key('keydown', 'KeyW'), false);
windowEvents.focus();
key('keydown', 'KeyW', true);
assert.equal(run('moveY'), 0, 'focus return must not restore held keys');
key('keydown', 'KeyW');
context.document.hidden = true;
documentEvents.visibilitychange();
assert.equal(run('pressedKeys.size'), 0);
assert.equal(ws.sent.at(-1), 'CMD,0.000,0.000,0.000,0,1');
assert.equal(key('keydown', 'KeyQ'), false);
context.document.hidden = false;
documentEvents.visibilitychange();
key('keydown', 'KeyW');
status(false);
assert.equal(run('pressedKeys.size'), 0);
assert.equal(run('moveX+moveY+rotX'), 0);
assert.equal(key('keyup', 'KeyW'), false);
status(true);
ws.close();
assert.equal(run('isController'), false);
assert.equal(element('connection').innerText, 'OFFLINE');
assert.equal(element('controlButton').disabled, true);
const afterClose = ws.sent.length;
heartbeat.callback();
assert.equal(ws.sent.length, afterClose, 'no heartbeat after disconnect');
console.log('PASS: spectator guards, claim/release, movement, one-shot kick, control loss, telemetry, disconnect');
console.log('PASS: WASD/Q/E combinations, touch coexistence, SPACE, repeat, blur, visibility, keyboard ownership');

console.log("PASS: single automatic claim, configured heartbeat, spectator/disconnect guards");
