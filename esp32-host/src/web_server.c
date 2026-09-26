#include "web_server.h"

#include <errno.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <sys/socket.h>
#include <unistd.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "app_config.h"

#define WEB_MAX_CLIENTS 8
#define STRINGIFY_VALUE_(value) #value
#define STRINGIFY_VALUE(value) STRINGIFY_VALUE_(value)

static const char *TAG = "WEB";
static portMUX_TYPE state_mux = portMUX_INITIALIZER_UNLOCKED;
static float command_vx, command_vy, command_omega;
static uint8_t command_kick;
static bool command_brake = true;
static int64_t last_command_us;
static int controller_fd = -1;
static int64_t controller_last_seen_us;
static esp_timer_handle_t lease_timer;
static bool lease_expired_log;
static quadmd_telemetry_t telemetry_snapshot;
static bool control_dirty = true;
static bool broadcast_pending;
/* Immutable after startup; all session operations run on the HTTP task. */
static httpd_handle_t server;

/* HTTP-task-owned queues include HTTP upgrade and WS control frames so partially
 * sent frames can never interleave. New telemetry is dropped while bytes remain. */
typedef struct {
    bool active;
    bool failed;
    int fd;
    size_t offset;
    size_t length;
    unsigned int failures;
    int64_t last_warning_us;
    uint8_t bytes[1024];
} web_tx_t;
static web_tx_t client_tx[WEB_MAX_CLIENTS];

static const char index_html[] =
"<!DOCTYPE html>"
"<html>"

"<head><meta charset='utf-8'>"

"<meta name='viewport' "
"content='width=device-width,"
"initial-scale=1,"
"maximum-scale=1,"
"user-scalable=no'>"

"<title>TauraBots</title>"

"<style>"

"*{"
"box-sizing:border-box;"
"touch-action:none;"
"}"

"body{"
"margin:0;"
"background:#101114;"
"color:#fff;"
"font-family:Arial,sans-serif;"
"text-align:center;"
"}"

"h1{"
"margin:15px 0 5px;"
"}"

".status{"
"font-size:14px;"
"margin-bottom:10px;"
"}"

".online{"
"color:#5cff7a;"
"}"

".offline{"
"color:#ff5757;"
"}"

".panel{"
"background:#1b1d22;"
"margin:10px;"
"padding:15px;"
"border-radius:15px;"
"}"

".controls{"
"display:flex;"
"justify-content:space-around;"
"align-items:center;"
"gap:15px;"
"}"

".joystick{"
"position:relative;"
"width:150px;"
"height:150px;"
"border-radius:50%;"
"background:#292c33;"
"border:2px solid #555;"
"}"

".knob{"
"position:absolute;"
"width:55px;"
"height:55px;"
"border-radius:50%;"
"background:#ddd;"
"left:47.5px;"
"top:47.5px;"
"}"

".label{"
"margin-top:8px;"
"font-size:14px;"
"color:#aaa;"
"}"

"input[type=range]{"
"width:85%;"
"}"

"button{"
"padding:15px 25px;"
"margin:7px;"
"border:0;"
"border-radius:10px;"
"font-weight:bold;"
"font-size:16px;"
"}"

".kick{"
"background:#e8b020;"
"}"

".stop{"
"background:#d72f2f;"
"color:white;"
"}"

".telemetry{"
"display:grid;"
"grid-template-columns:1fr 1fr;"
"gap:8px;"
"text-align:left;"
"}"

".value{"
"color:#5cc8ff;"
"font-weight:bold;"
"}"

"button:disabled,.joystick.disabled{opacity:.4;pointer-events:none;}"
".keyboard-help{display:none;color:#aaa;font-size:13px;}"
"@media (hover:hover) and (pointer:fine){.keyboard-help{display:block;}}"
"</style>"

"</head>"

"<body>"

"<h1>TAURABOTS</h1>"

"<div class='status'>"
"WebSocket: "
"<span id='connection' class='offline'>OFFLINE</span>"
"</div>"

"<div class='status'><span id='role'>ESPECTADOR</span></div>"
"<button id='controlButton' disabled onclick='toggleControl()'>ASSUMIR CONTROLE</button>"
"<p class='keyboard-help'>WASD: movimento | Q/E: rota&ccedil;&atilde;o | Espa&ccedil;o: STOP</p>"
"<div class='panel'>"

"<div class='controls'>"

"<div>"

"<div id='moveJoy' class='joystick'>"
"<div id='moveKnob' class='knob'></div>"
"</div>"

"<div class='label'>MOVIMENTO</div>"

"</div>"

"<div>"

"<div id='rotJoy' class='joystick'>"
"<div id='rotKnob' class='knob'></div>"
"</div>"

"<div class='label'>ROTACAO</div>"

"</div>"

"</div>"
"</div>"

"<div class='panel'>"

"<p>"
"Velocidade linear: "
"<span id='linearText'>0.20</span> m/s"
"</p>"

"<input "
"id='linear' "
"type='range' "
"min='0.05' "
"max='2.00' "
"step='0.05' "
"value='0.20'>"

"<p>"
"Velocidade angular: "
"<span id='angularText'>1.00</span> rad/s"
"</p>"

"<input "
"id='angular' "
"type='range' "
"min='0.10' "
"max='10.00' "
"step='0.10' "
"value='1.00'>"

"<p>"
"Kicker: "
"<span id='kickText'>50</span>%"
"</p>"

"<input "
"id='kickPower' "
"type='range' "
"min='0' "
"max='100' "
"step='1' "
"value='50'>"

"<br>"

"<button class='kick' onclick='kick()'>"
"KICK"
"</button>"

"<button class='stop' onclick='emergencyStop()'>"
"EMERGENCY STOP"
"</button>"

"</div>"

"<div class='panel telemetry'>"

"<div>Bateria</div>"
"<div class='value'>"
"<span id='battery'>--</span> V"
"</div>"

"<div>RPM M1</div>"
"<div class='value' id='rpm1'>--</div>"

"<div>RPM M2</div>"
"<div class='value' id='rpm2'>--</div>"

"<div>RPM M3</div>"
"<div class='value' id='rpm3'>--</div>"

"<div>RPM M4</div>"
"<div class='value' id='rpm4'>--</div>"

"<div>Quad-MD</div>"
"<div class='value' id='comm'>--</div>"

"<div>Watchdog</div>"
"<div class='value' id='watchdog'>--</div>"

"<div>Sequence</div><div class='value' id='sequence'>--</div>"
"<div>Fault</div>"
"<div class='value' id='fault'>--</div>"

"</div>"

"<script>"

// ============================================================
// Estado
// ============================================================

"let ws=null;"
"let isController=false;"
"let occupied=null,claimPending=false,autoClaim=true;"
"let claimTimer=null,claimCooldown=null;"
"function cancelClaimTimers(){"
"clearTimeout(claimTimer);clearTimeout(claimCooldown);"
"claimTimer=claimCooldown=null;claimPending=false;"
"}"
"function scheduleClaim(){"
"if(!autoClaim||occupied===true||isController||claimPending||claimTimer!==null)return;"
"const socket=ws;"
"claimTimer=setTimeout(()=>{claimTimer=null;if(ws===socket)claimControl();},100);"
"}"
"function claimControl(){"
"if(!ws||ws.readyState!==WebSocket.OPEN||isController||claimPending)return;"
"claimPending=true;ws.send('CLAIM_CONTROL');"
"claimCooldown=setTimeout(()=>{"
"claimCooldown=null;claimPending=false;"
"if(occupied===false)scheduleClaim();"
"},300);"
"}"
"const joystickResets=[];"
"const pressedKeys=new Set();"
"const controlKeys=new Set(['KeyW','KeyA','KeyS','KeyD','KeyQ','KeyE','Space']);"
"let joystickX=0,joystickY=0,joystickRotation=0;"
"let pageFocused=document.hasFocus();"
"function canControl(){return isController&&pageFocused&&!document.hidden;}"
"function updateMotion(){"
"const key=code=>pressedKeys.has(code)?1:0;"
"const x=joystickX+key('KeyD')-key('KeyA');"
"const y=joystickY+key('KeyW')-key('KeyS');"
"const scale=Math.max(1,Math.hypot(x,y));"
"moveX=x/scale;moveY=y/scale;"
"rotX=Math.max(-1,Math.min(1,joystickRotation+key('KeyQ')-key('KeyE')));"
"if(moveX!==0||moveY!==0||rotX!==0)emergency=false;"
"}"
"function resetControls(){"
"pressedKeys.clear();joystickX=joystickY=joystickRotation=0;"
"moveX=moveY=rotX=kickPending=0;emergency=true;"
"joystickResets.forEach(reset=>reset());"
"}"
"function setControl(value){"
"if(isController!==value||!value){resetControls();}"
"isController=value;"
"if(value)cancelClaimTimers();"
"document.getElementById('role').innerText=value?'CONTROLADOR':'ESPECTADOR';"
"document.getElementById('controlButton').innerText=value?'LIBERAR CONTROLE':'ASSUMIR CONTROLE';"
"document.querySelectorAll('input,.kick,.stop').forEach(el=>el.disabled=!value);"
"document.querySelectorAll('.joystick').forEach(el=>el.classList.toggle('disabled',!value));"
"}"
"function toggleControl(){"
"if(!ws||ws.readyState!==1)return;"
"if(isController){"
"autoClaim=false;cancelClaimTimers();ws.send('RELEASE_CONTROL');setControl(false);"
"}else{autoClaim=true;claimControl();}"
"}"
"window.addEventListener('pagehide',()=>{"
"if(ws&&ws.readyState===1){if(isController)ws.send('RELEASE_CONTROL');ws.close();}"
"setControl(false);"
"});"

"let moveX=0;"
"let moveY=0;"
"let rotX=0;"

"let kickPending=0;"
"let emergency=false;"

// ============================================================
// Sliders
// ============================================================

"const linear="
"document.getElementById('linear');"

"const angular="
"document.getElementById('angular');"

"const kickPower="
"document.getElementById('kickPower');"

"linear.oninput=()=>{"

"document.getElementById('linearText').innerText="
"parseFloat(linear.value).toFixed(2);"

"};"

"angular.oninput=()=>{"

"document.getElementById('angularText').innerText="
"parseFloat(angular.value).toFixed(2);"

"};"

"kickPower.oninput=()=>{"

"document.getElementById('kickText').innerText="
"kickPower.value;"

"};"

// ============================================================
// WebSocket
// ============================================================

"function connectWS(){"

"ws=new WebSocket("
"'ws://'+location.host+'/ws'"
");"

"ws.onopen=()=>{"
"setControl(false);document.getElementById('controlButton').disabled=false;"
"cancelClaimTimers();occupied=null;autoClaim=true;scheduleClaim();"

"document.getElementById('connection').innerText="
"'ONLINE';"

"document.getElementById('connection').className="
"'online';"

"};"

"ws.onclose=()=>{"
"cancelClaimTimers();occupied=null;"
"setControl(false);document.getElementById('controlButton').disabled=true;"

"document.getElementById('connection').innerText="
"'OFFLINE';"

"document.getElementById('connection').className="
"'offline';"

"setTimeout("
"connectWS,"
"1000"
");"

"};"

"ws.onerror=()=>{"
"setControl(false);"
"ws.close();"
"};"

"ws.onmessage=(event)=>{"

"let d;"

"try{"
"d=JSON.parse(event.data);"
"}"
"catch(e){"
"return;"
"}"

"if(d.type==='control'){"
"occupied=d.occupied;setControl(d.controller===true);"
"if(occupied===true){clearTimeout(claimTimer);claimTimer=null;}"
"else if(occupied===false){scheduleClaim();}"
"return;}"
"document.getElementById('sequence').innerText=d.sequence;"
"if(d.battery!==undefined){"

"document.getElementById('battery').innerText="
"d.battery.toFixed(3);"

"}"

"if(d.rpm){"

"document.getElementById('rpm1').innerText="
"d.rpm[0].toFixed(1);"

"document.getElementById('rpm2').innerText="
"d.rpm[1].toFixed(1);"

"document.getElementById('rpm3').innerText="
"d.rpm[2].toFixed(1);"

"document.getElementById('rpm4').innerText="
"d.rpm[3].toFixed(1);"

"}"

"document.getElementById('comm').innerText="
"d.comm?'OK':'LOST';"

"document.getElementById('watchdog').innerText="
"d.watchdog?'OK':'FAULT';"

"document.getElementById('fault').innerText="
"'0x'+"
"d.fault.toString(16)"
".padStart(2,'0')"
".toUpperCase();"

"};"

"}"

"connectWS();"

// ============================================================
// Joystick
// ============================================================

"function setupJoystick("
"baseId,"
"knobId,"
"callback,"
"rotationOnly"
"){"

"const base="
"document.getElementById(baseId);"

"const knob="
"document.getElementById(knobId);"

"let active=false;"

"function update(e){"
"if(!canControl())return;"

"const r="
"base.getBoundingClientRect();"

"const cx="
"r.left+r.width/2;"

"const cy="
"r.top+r.height/2;"

"let dx="
"e.clientX-cx;"

"let dy="
"e.clientY-cy;"

"if(rotationOnly){"
"dy=0;"
"}"

"const radius="
"r.width/2-30;"

"const len="
"Math.sqrt("
"dx*dx+dy*dy"
");"

"if(len>radius){"

"dx="
"dx/len*radius;"

"dy="
"dy/len*radius;"

"}"

"knob.style.transform="
"`translate(${dx}px,${dy}px)`;"

"callback("
"dx/radius,"
"-dy/radius"
");"

"}"

"base.addEventListener("
"'pointerdown',"
"e=>{"

"if(!canControl())return;"
"active=true;"

"base.setPointerCapture("
"e.pointerId"
");"

"update(e);"

"});"

"base.addEventListener("
"'pointermove',"
"e=>{"

"if(active){"
"update(e);"
"}"

"});"

"function release(){"

"active=false;"

"knob.style.transform="
"'translate(0px,0px)';"

"callback(0,0);"

"}"

"joystickResets.push(release);"
"base.addEventListener('lostpointercapture',release);"
"base.addEventListener("
"'pointerup',"
"release"
");"

"base.addEventListener("
"'pointercancel',"
"release"
");"

"}"

// ============================================================
// Configura joysticks
// ============================================================

"setupJoystick("
"'moveJoy',"
"'moveKnob',"
"(x,y)=>{"

"joystickX=x;"
"joystickY=y;"
"updateMotion();"

"},"
"false"
");"

"setupJoystick("
"'rotJoy',"
"'rotKnob',"
"(x,y)=>{"

"joystickRotation=x;"
"updateMotion();"

"},"
"true"
");"

// ============================================================
// Kick
// ============================================================

"function kick(){"
"if(!canControl())return;"

"kickPending="
"parseInt("
"kickPower.value"
");"

"}"

// ============================================================
// Emergency stop
// ============================================================

"function emergencyStop(){"
"if(!canControl())return;resetControls();"

"moveX=0;"
"moveY=0;"
"rotX=0;"

"kickPending=0;"

"emergency=true;"

"}"

// ============================================================
// TX comando
// ============================================================

"function sendCommand(){"

"if("
"!isController || !ws || "
"ws.readyState!==1"
"){"

"return;"

"}"

"const maxLinear="
"parseFloat(linear.value);"

"const maxAngular="
"parseFloat(angular.value);"

"const vx="
"moveX*maxLinear;"

"const vy="
"moveY*maxLinear;"

"const omega="
"rotX*maxAngular;"

"const stationary="

"Math.abs(vx)<0.001 && "
"Math.abs(vy)<0.001 && "
"Math.abs(omega)<0.001;"

"const brake="
"(emergency||stationary)"
"?1:0;"

"const msg="
"`CMD,"
"${vx.toFixed(3)},"
"${vy.toFixed(3)},"
"${omega.toFixed(3)},"
"${kickPending},"
"${brake}`;"

"ws.send(msg);"

"kickPending=0;"

"}"

// Keyboard and touch feed the same motion state; sendCommand remains the only CMD sender.
"window.addEventListener('keydown',e=>{"
"if(!canControl()||!controlKeys.has(e.code))return;"
"e.preventDefault();"
"if(e.repeat)return;"
"if(e.code==='Space'){emergencyStop();pressedKeys.add('Space');return;}"
"if(pressedKeys.has('Space'))return;"
"pressedKeys.add(e.code);updateMotion();"
"});"
"window.addEventListener('keyup',e=>{"
"if(!canControl()||!controlKeys.has(e.code))return;"
"e.preventDefault();pressedKeys.delete(e.code);updateMotion();"
"});"
"function resetOnFocusLoss(){resetControls();sendCommand();}"
"window.addEventListener('blur',()=>{pageFocused=false;resetOnFocusLoss();});"
"window.addEventListener('focus',()=>{pageFocused=true;});"
"document.addEventListener('visibilitychange',()=>{"
"pageFocused=!document.hidden&&document.hasFocus();"
"resetOnFocusLoss();"
"});"

"function sendHeartbeat(){"
"if(ws&&ws.readyState===WebSocket.OPEN&&isController)ws.send('HEARTBEAT');"
"}"
"setInterval(sendHeartbeat," STRINGIFY_VALUE(CONTROLLER_HEARTBEAT_MS) ");"

// Browser -> ESP32 @20 Hz
"setInterval("
"sendCommand,"
"50"
");"

"setControl(false);"
"</script>"

"</body>"
"</html>";


/* Caller holds state_mux. No network operations inside critical sections. */
static void safe_command_locked(void)
{
    command_vx = command_vy = command_omega = 0.0f;
    command_kick = 0;
    command_brake = true;
    last_command_us = 0;
}

/* All helpers ending in _locked require state_mux. */
static void clear_controller_locked(void)
{
    controller_fd = -1;
    controller_last_seen_us = 0;
    safe_command_locked();
    control_dirty = true;
}

static void expire_controller_locked(int64_t now)
{
    if (controller_fd >= 0 &&
        now - controller_last_seen_us >= CONTROLLER_LEASE_TIMEOUT_MS * 1000LL) {
        clear_controller_locked();
        control_dirty = true;
        lease_expired_log = true;
    }
}

/* Independent of blocked HTTP sends/receives. No network or logging in timer task. */
static void lease_timer_callback(void *arg)
{
    (void)arg;
    portENTER_CRITICAL(&state_mux);
    expire_controller_locked(esp_timer_get_time());
    portEXIT_CRITICAL(&state_mux);
}

static void release_controller(int fd)
{
    portENTER_CRITICAL(&state_mux);
    const bool released = controller_fd >= 0 && controller_fd == fd;
    if (released) {
        clear_controller_locked();
    }
    portEXIT_CRITICAL(&state_mux);
    if (released) {
        ESP_LOGI(TAG, "Controle liberado fd=%d", fd);
    }
}

/* HTTP task only: session lookups must be serialized with HTTPD session cleanup. */
static bool controller_is_valid(void)
{
    portENTER_CRITICAL(&state_mux);
    expire_controller_locked(esp_timer_get_time());
    const int fd = controller_fd;
    portEXIT_CRITICAL(&state_mux);
    if (fd < 0) {
        return false;
    }
    if (httpd_ws_get_fd_info(server, fd) != HTTPD_WS_CLIENT_WEBSOCKET) {
        release_controller(fd);
        return false;
    }
    portENTER_CRITICAL(&state_mux);
    expire_controller_locked(esp_timer_get_time());
    const bool valid = controller_fd == fd;
    portEXIT_CRITICAL(&state_mux);
    return valid;
}

static web_tx_t *find_tx(int fd)
{
    for (size_t i = 0; i < WEB_MAX_CLIENTS; ++i) {
        if (client_tx[i].active && client_tx[i].fd == fd) {
            return &client_tx[i];
        }
    }
    return NULL;
}

static void session_close(httpd_handle_t handle, int fd)
{
    (void)handle;
    release_controller(fd);
    web_tx_t *tx = find_tx(fd);
    if (tx != NULL) {
        memset(tx, 0, sizeof(*tx));
    }
    close(fd); /* HTTPD does not close sockets when close_fn is installed. */
}

static void fail_tx(web_tx_t *tx)
{
    if (!tx->failed) {
        tx->failed = true;
        release_controller(tx->fd);
        shutdown(tx->fd, SHUT_RDWR); /* HTTPD owns deletion and descriptor reuse. */
    }
}

/* One nonblocking write per attempt: no slow peer stalls all other clients.
 * Preserve partial bytes on EAGAIN; discarding them would corrupt the WS stream. */
static bool flush_tx(web_tx_t *tx)
{
    if (tx->failed) {
        return false;
    }
    if (tx->offset == tx->length) {
        tx->offset = tx->length = 0;
        return true;
    }
    const int written = send(tx->fd, tx->bytes + tx->offset,
                             tx->length - tx->offset, MSG_DONTWAIT);
    if (written > 0) {
        tx->offset += (size_t)written;
        tx->failures = 0;
        if (tx->offset == tx->length) {
            tx->offset = tx->length = 0;
            return true;
        }
        return false;
    }
    const int error = written < 0 ? errno : ECONNRESET;
    const httpd_ws_client_info_t info = httpd_ws_get_fd_info(server, tx->fd);
    if (info == HTTPD_WS_CLIENT_INVALID || error == ECONNRESET ||
        error == EPIPE || error == ENOTCONN || error == EBADF) {
        ESP_LOGW(TAG, "fd=%d socket encerrado, limpando sessao (errno=%d)", tx->fd, error);
        fail_tx(tx);
        return false;
    }
    /* Congestion/resource pressure is not proof that the controller died.
     * Lease, RX errors and TCP keepalive still detect a lost peer. */
    if (tx->failures < UINT32_MAX) {
        ++tx->failures;
    }
    const int64_t now = esp_timer_get_time();
    if (tx->last_warning_us == 0 || now - tx->last_warning_us >= 5000000LL) {
        tx->last_warning_us = now;
        ESP_LOGW(TAG, "TX pendente fd=%d errno=%d falhas=%u; telemetria nova descartavel",
                 tx->fd, error, tx->failures);
    }
    return false;
}

/* Used by HTTPD for every outgoing byte on this upgraded session, including
 * PONG/CLOSE. Returning length means the bytes were accepted into our bounded
 * queue, not necessarily transmitted over Wi-Fi yet. */
static int buffered_send(httpd_handle_t handle, int fd, const char *buf, size_t len, int flags)
{
    (void)handle;
    (void)flags;
    web_tx_t *tx = find_tx(fd);
    if (tx == NULL || tx->failed) {
        return HTTPD_SOCK_ERR_FAIL;
    }
    if (tx->offset > 0) {
        memmove(tx->bytes, tx->bytes + tx->offset, tx->length - tx->offset);
        tx->length -= tx->offset;
        tx->offset = 0;
    }
    if (len > sizeof(tx->bytes) - tx->length) {
        /* Only protocol/control flooding can fill this: telemetry checks space
         * before handing a whole frame to HTTPD. Never truncate a frame. */
        return HTTPD_SOCK_ERR_FAIL;
    }
    memcpy(tx->bytes + tx->length, buf, len);
    tx->length += len;
    (void)flush_tx(tx);
    return tx->failed ? HTTPD_SOCK_ERR_FAIL : (int)len;
}

static esp_err_t init_tx(int fd)
{
    for (size_t i = 0; i < WEB_MAX_CLIENTS; ++i) {
        if (!client_tx[i].active) {
            client_tx[i] = (web_tx_t){ .active = true, .fd = fd };
            const esp_err_t err = httpd_sess_set_send_override(server, fd, buffered_send);
            if (err != ESP_OK) {
                memset(&client_tx[i], 0, sizeof(client_tx[i]));
            }
            return err;
        }
    }
    return ESP_ERR_NO_MEM;
}

static bool send_text(int fd, const char *text)
{
    if (httpd_ws_get_fd_info(server, fd) != HTTPD_WS_CLIENT_WEBSOCKET) {
        release_controller(fd);
        return false;
    }
    web_tx_t *tx = find_tx(fd);
    if (tx == NULL || !flush_tx(tx)) {
        return false; /* Drop this NEW frame, retain any already partially sent. */
    }
    const size_t len = strlen(text);
    if (len + 10 > sizeof(tx->bytes)) {
        return false;
    }
    httpd_ws_frame_t frame = {
        .type = HTTPD_WS_TYPE_TEXT,
        .payload = (uint8_t *)text,
        .len = len,
    };
    return httpd_ws_send_frame_async(server, fd, &frame) == ESP_OK;
}

/* HTTP task only. Status is personalized: controller=true only for its owner. */
static void broadcast_control(void)
{
    int clients[WEB_MAX_CLIENTS];
    size_t count = WEB_MAX_CLIENTS;
    if (httpd_get_client_list(server, &count, clients) != ESP_OK) {
        return;
    }
    portENTER_CRITICAL(&state_mux);
    control_dirty = false;
    portEXIT_CRITICAL(&state_mux);
    for (size_t i = 0; i < count; ++i) {
        const int fd = clients[i];
        if (httpd_ws_get_fd_info(server, fd) != HTTPD_WS_CLIENT_WEBSOCKET) {
            continue;
        }
        portENTER_CRITICAL(&state_mux);
        const int owner = controller_fd;
        portEXIT_CRITICAL(&state_mux);
        char json[96];
        snprintf(json, sizeof(json),
                 "{\"type\":\"control\",\"controller\":%s,\"occupied\":%s}",
                 owner == fd ? "true" : "false", owner >= 0 ? "true" : "false");
        if (!send_text(fd, json)) {
            portENTER_CRITICAL(&state_mux);
            control_dirty = true; /* Retry control status; telemetry is disposable. */
            portEXIT_CRITICAL(&state_mux);
        }
    }
}

static esp_err_t root_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html; charset=utf-8");
    /* Leave the limited client slots available for persistent WebSockets. */
    httpd_resp_set_hdr(req, "Connection", "close");
    return httpd_resp_send(req, index_html, HTTPD_RESP_USE_STRLEN);
}

static void process_command(int fd, const char *payload)
{
    float vx, vy, omega;
    unsigned int kick, brake;
    int consumed = 0;
    if (sscanf(payload, "CMD,%f,%f,%f,%u,%u%n",
               &vx, &vy, &omega, &kick, &brake, &consumed) != 5 ||
        payload[consumed] != '\0' || !isfinite(vx) || !isfinite(vy) ||
        !isfinite(omega) || kick > 100U || brake > 1U) {
        return;
    }
    vx = fmaxf(-3.0f, fminf(3.0f, vx));
    vy = fmaxf(-3.0f, fminf(3.0f, vy));
    omega = fmaxf(-15.0f, fminf(15.0f, omega));
    portENTER_CRITICAL(&state_mux);
    const int64_t now = esp_timer_get_time();
    expire_controller_locked(now);
    if (controller_fd == fd) {
        controller_last_seen_us = now;
        command_vx = vx;
        command_vy = vy;
        command_omega = omega;
        /* A zero packet must not erase a pending one-shot event. */
        if (kick > 0U) {
            command_kick = (uint8_t)kick;
        }
        command_brake = brake != 0U;
        last_command_us = now;
    }
    portEXIT_CRITICAL(&state_mux);
}

static esp_err_t websocket_handler(httpd_req_t *req)
{
    const int fd = httpd_req_to_sockfd(req);
    if (req->method == HTTP_GET) {
        const esp_err_t tx_err = init_tx(fd);
        if (tx_err != ESP_OK) {
            return tx_err;
        }
        /* HTTPD completes the upgrade after this handler returns. The next queued
         * broadcast discovers the new session, without retaining its descriptor. */
        portENTER_CRITICAL(&state_mux);
        control_dirty = true;
        portEXIT_CRITICAL(&state_mux);
        ESP_LOGI(TAG, "WS conectado fd=%d", fd);
        return ESP_OK;
    }
    httpd_ws_frame_t frame = {0};
    esp_err_t err = httpd_ws_recv_frame(req, &frame, 0);
    if (err != ESP_OK) {
        release_controller(fd);
        return err;
    }
    char payload[128];
    if (frame.len >= sizeof(payload)) {
        release_controller(fd);
        return ESP_ERR_INVALID_SIZE;
    }
    frame.payload = (uint8_t *)payload;
    /* A second receive on a zero-length frame would read the next frame header. */
    if (frame.len > 0) {
        err = httpd_ws_recv_frame(req, &frame, sizeof(payload) - 1);
        if (err != ESP_OK) {
            release_controller(fd);
            return err;
        }
    }
    if (frame.type == HTTPD_WS_TYPE_CLOSE) {
        release_controller(fd);
        broadcast_control();
        return ESP_OK;
    }
    if (frame.type != HTTPD_WS_TYPE_TEXT || !frame.final ||
        memchr(payload, '\0', frame.len) != NULL) {
        return ESP_OK;
    }
    payload[frame.len] = '\0';
    if (strcmp(payload, "CLAIM_CONTROL") == 0) {
        ESP_LOGI(TAG, "fd=%d solicitou controle", fd);
        (void)controller_is_valid();
        portENTER_CRITICAL(&state_mux);
        const int64_t now = esp_timer_get_time();
        expire_controller_locked(now);
        if (controller_fd == -1) {
            safe_command_locked();
            controller_fd = fd;
            control_dirty = true;
        }
        const bool granted = controller_fd == fd;
        if (granted) {
            controller_last_seen_us = now;
        }
        portEXIT_CRITICAL(&state_mux);
        ESP_LOGI(TAG, "fd=%d %s", fd,
                 granted ? "agora e CONTROLADOR" : "claim negado, controller ativo");
        broadcast_control();
    } else if (strcmp(payload, "RELEASE_CONTROL") == 0) {
        release_controller(fd);
        broadcast_control();
    } else if (strcmp(payload, "HEARTBEAT") == 0) {
        portENTER_CRITICAL(&state_mux);
        const int64_t now = esp_timer_get_time();
        expire_controller_locked(now);
        if (controller_fd == fd) {
            controller_last_seen_us = now;
        }
        portEXIT_CRITICAL(&state_mux);
    } else {
        process_command(fd, payload);
    }
    return ESP_OK;
}

void web_server_get_command(web_command_t *command)
{
    if (command == NULL) {
        return;
    }
    portENTER_CRITICAL(&state_mux);
    const int64_t now = esp_timer_get_time();
    expire_controller_locked(now);
    const int64_t age = last_command_us > 0 ? (now - last_command_us) / 1000 : INT64_MAX;
    command->age_ms = age >= UINT32_MAX ? UINT32_MAX : (uint32_t)age;
    command->connected = controller_fd >= 0 && age <= WEB_COMMAND_TIMEOUT_MS;
    if (!command->connected) {
        safe_command_locked();
    }
    command->vx = command_vx;
    command->vy = command_vy;
    command->omega = command_omega;
    command->kick_power = command_kick;
    command->brake = command_brake;
    command_kick = 0;
    portEXIT_CRITICAL(&state_mux);
}

void web_server_update_telemetry(const quadmd_telemetry_t *telemetry)
{
    if (telemetry == NULL) {
        return;
    }
    portENTER_CRITICAL(&state_mux);
    telemetry_snapshot = *telemetry;
    portEXIT_CRITICAL(&state_mux);
}

/* Executed by HTTPD, serialized with handlers, session cleanup and other sends. */
static void broadcast_work(void *arg)
{
    (void)arg;
    for (size_t i = 0; i < WEB_MAX_CLIENTS; ++i) {
        if (client_tx[i].active) {
            (void)flush_tx(&client_tx[i]);
        }
    }
    (void)controller_is_valid();
    portENTER_CRITICAL(&state_mux);
    const bool dirty = control_dirty;
    const quadmd_telemetry_t t = telemetry_snapshot;
    portEXIT_CRITICAL(&state_mux);
    if (dirty) {
        broadcast_control();
    }
    char json[384];
    const int len = snprintf(json, sizeof(json),
        "{\"valid\":%s,\"battery\":%.3f,\"rpm\":[%.1f,%.1f,%.1f,%.1f],"
        "\"cmd\":[%d,%d,%d,%d],\"comm\":%s,\"watchdog\":%s,\"fault\":%u,\"sequence\":%lu}",
        t.valid ? "true" : "false", t.battery_voltage,
        t.rpm[0], t.rpm[1], t.rpm[2], t.rpm[3],
        t.motor_command[0], t.motor_command[1], t.motor_command[2], t.motor_command[3],
        t.communication_ok ? "true" : "false", t.watchdog_ok ? "true" : "false",
        (unsigned int)t.fault_status, (unsigned long)t.last_command_sequence);
    int clients[WEB_MAX_CLIENTS];
    size_t count = WEB_MAX_CLIENTS;
    if (len > 0 && len < (int)sizeof(json) &&
        httpd_get_client_list(server, &count, clients) == ESP_OK) {
        for (size_t i = 0; i < count; ++i) {
            if (httpd_ws_get_fd_info(server, clients[i]) == HTTPD_WS_CLIENT_WEBSOCKET) {
                send_text(clients[i], json);
            }
        }
    }
    portENTER_CRITICAL(&state_mux);
    broadcast_pending = false;
    portEXIT_CRITICAL(&state_mux);
}

static void telemetry_web_task(void *arg)
{
    (void)arg;
    TickType_t period = pdMS_TO_TICKS(1000 / WEB_TELEMETRY_HZ);
    if (period == 0) {
        period = 1;
    }
    for (;;) {
        vTaskDelay(period);
        portENTER_CRITICAL(&state_mux);
        const bool expired = lease_expired_log;
        lease_expired_log = false;
        const bool enqueue = !broadcast_pending;
        if (enqueue) {
            broadcast_pending = true;
        }
        portEXIT_CRITICAL(&state_mux);
        if (expired) {
            ESP_LOGI(TAG, "Controller lease expirou; controle liberado");
        }
        /* At most one pending broadcast: slow clients cannot grow the work queue. */
        if (enqueue && httpd_queue_work(server, broadcast_work, NULL) != ESP_OK) {
            portENTER_CRITICAL(&state_mux);
            broadcast_pending = false;
            portEXIT_CRITICAL(&state_mux);
        }
    }
}

esp_err_t web_server_start(void)
{
    if (server != NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.max_open_sockets = WEB_MAX_CLIENTS;
    config.close_fn = session_close;
    config.recv_wait_timeout = 1;
    config.send_wait_timeout = 1;
    /* Do not evict the controller to make room for another HTTP connection. */
    config.lru_purge_enable = false;
    config.keep_alive_enable = true;
    config.keep_alive_idle = 5;
    config.keep_alive_interval = 1;
    config.keep_alive_count = 3;
    esp_err_t err = httpd_start(&server, &config);
    if (err != ESP_OK) {
        server = NULL;
        return err;
    }
    const httpd_uri_t root_uri = {
        .uri = "/", .method = HTTP_GET, .handler = root_handler,
    };
    const httpd_uri_t websocket_uri = {
        .uri = "/ws", .method = HTTP_GET, .handler = websocket_handler,
        .is_websocket = true,
    };
    err = httpd_register_uri_handler(server, &root_uri);
    if (err == ESP_OK) {
        err = httpd_register_uri_handler(server, &websocket_uri);
    }
    const esp_timer_create_args_t timer_args = {
        .callback = lease_timer_callback,
        .name = "controller_lease",
    };
    if (err == ESP_OK) {
        err = esp_timer_create(&timer_args, &lease_timer);
    }
    if (err == ESP_OK) {
        err = esp_timer_start_periodic(
            lease_timer,
            CONTROLLER_LEASE_CHECK_PERIOD_US
        );
    }
    if (err == ESP_OK && xTaskCreatePinnedToCore(telemetry_web_task,
            "web_telemetry", 4096, NULL, 4, NULL, 0) != pdPASS) {
        err = ESP_ERR_NO_MEM;
    }
    if (err != ESP_OK) {
        if (lease_timer != NULL) {
            esp_timer_stop(lease_timer);
            esp_timer_delete(lease_timer);
            lease_timer = NULL;
        }
        httpd_stop(server);
        server = NULL;
        return err;
    }
    ESP_LOGI(TAG, "HTTP/WebSocket started: %d clients", WEB_MAX_CLIENTS);
    return ESP_OK;
}
