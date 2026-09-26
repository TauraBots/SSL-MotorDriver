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
#include "wifi_sta.h"
#include "web_assets.h"

#define WEB_MAX_CLIENTS 8

static const char *TAG = "WEB";
static portMUX_TYPE state_mux = portMUX_INITIALIZER_UNLOCKED;
static float command_vx, command_vy, command_omega;
static uint8_t command_kick;
static bool command_brake = true;
static int64_t last_command_us;
static int active_fd = -1;
static quadmd_telemetry_t telemetry_snapshot;
static bool active_dirty = true;
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

/* Caller holds state_mux. No network operations inside critical sections. */
static void safe_command_locked(void)
{
    command_vx = command_vy = command_omega = 0.0f;
    command_kick = 0;
    command_brake = true;
    last_command_us = 0;
}

/* All helpers ending in _locked require state_mux. */
static void clear_active_locked(void)
{
    active_fd = -1;
    safe_command_locked();
    active_dirty = true;
}

static void release_active(int fd)
{
    portENTER_CRITICAL(&state_mux);
    const bool released = active_fd == fd;
    if (released) {
        clear_active_locked();
    }
    portEXIT_CRITICAL(&state_mux);
    if (released) {
        ESP_LOGI(TAG, "Controle liberado fd=%d", fd);
    }
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
    release_active(fd);
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
        release_active(tx->fd);
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
    /* Congestion/resource pressure is not proof that the client died.
     * RX errors and TCP keepalive still detect a lost peer. */
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
        release_active(fd);
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

/* HTTP task only. Status is personalized for every connected WebSocket. */
static void broadcast_active(void)
{
    int clients[WEB_MAX_CLIENTS];
    size_t count = WEB_MAX_CLIENTS;
    if (httpd_get_client_list(server, &count, clients) != ESP_OK) {
        return;
    }
    portENTER_CRITICAL(&state_mux);
    active_dirty = false;
    portEXIT_CRITICAL(&state_mux);
    for (size_t i = 0; i < count; ++i) {
        const int fd = clients[i];
        if (httpd_ws_get_fd_info(server, fd) != HTTPD_WS_CLIENT_WEBSOCKET) {
            continue;
        }
        portENTER_CRITICAL(&state_mux);
        const int owner = active_fd;
        portEXIT_CRITICAL(&state_mux);
        char json[48];
        snprintf(json, sizeof(json),
                 "{\"type\":\"active\",\"active\":%s}",
                 owner == fd ? "true" : "false");
        if (!send_text(fd, json)) {
            portENTER_CRITICAL(&state_mux);
            active_dirty = true; /* Retry active status; telemetry is disposable. */
            portEXIT_CRITICAL(&state_mux);
        }
    }
}

static esp_err_t send_web_asset(httpd_req_t *req, const char *content_type,
                                const unsigned char *data, size_t length)
{
    httpd_resp_set_type(req, content_type);
    /* The UI and WebSocket schema ship in the same firmware image. Never let a
     * cached page run against a newer protocol after an OTA/serial update. */
    httpd_resp_set_hdr(req, "Cache-Control",
                       "no-store, no-cache, must-revalidate, max-age=0");
    httpd_resp_set_hdr(req, "Pragma", "no-cache");
    /* Leave the limited client slots available for persistent WebSockets. */
    httpd_resp_set_hdr(req, "Connection", "close");
    return httpd_resp_send(req, (const char *)data, (ssize_t)length);
}

static esp_err_t root_handler(httpd_req_t *req)
{
    return send_web_asset(req, "text/html; charset=utf-8",
                          web_index_html, WEB_INDEX_HTML_LEN);
}

static esp_err_t style_handler(httpd_req_t *req)
{
    return send_web_asset(req, "text/css; charset=utf-8",
                          web_style_css, WEB_STYLE_CSS_LEN);
}

static esp_err_t app_handler(httpd_req_t *req)
{
    return send_web_asset(req, "application/javascript; charset=utf-8",
                          web_app_js, WEB_APP_JS_LEN);
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
    if (active_fd == fd) {
        const int64_t now = esp_timer_get_time();
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

/* Transport-independent parser for the public WebSocket command API. */
static void process_websocket_message(int fd, const char *payload)
{
    if (strcmp(payload, "TAKE_CONTROL") == 0) {
        portENTER_CRITICAL(&state_mux);
        const int previous_fd = active_fd;
        if (previous_fd != fd) {
            safe_command_locked();
            active_fd = fd;
            active_dirty = true;
        }
        portEXIT_CRITICAL(&state_mux);
        if (previous_fd < 0) {
            ESP_LOGI(TAG, "fd=%d assumiu controle", fd);
        } else if (previous_fd != fd) {
            ESP_LOGI(TAG, "controle transferido fd=%d -> fd=%d", previous_fd, fd);
        }
        broadcast_active();
    } else if (strcmp(payload, "RELEASE_CONTROL") == 0) {
        release_active(fd);
        broadcast_active();
    } else if (strcmp(payload, "EMERGENCY_STOP") == 0) {
        portENTER_CRITICAL(&state_mux);
        clear_active_locked();
        portEXIT_CRITICAL(&state_mux);
        ESP_LOGW(TAG, "parada de emergencia fd=%d", fd);
        broadcast_active();
    } else {
        process_command(fd, payload);
    }
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
        active_dirty = true;
        portEXIT_CRITICAL(&state_mux);
        ESP_LOGI(TAG, "WS conectado fd=%d", fd);
        return ESP_OK;
    }
    httpd_ws_frame_t frame = {0};
    esp_err_t err = httpd_ws_recv_frame(req, &frame, 0);
    if (err != ESP_OK) {
        release_active(fd);
        return err;
    }
    char payload[128];
    if (frame.len >= sizeof(payload)) {
        release_active(fd);
        return ESP_ERR_INVALID_SIZE;
    }
    frame.payload = (uint8_t *)payload;
    /* A second receive on a zero-length frame would read the next frame header. */
    if (frame.len > 0) {
        err = httpd_ws_recv_frame(req, &frame, sizeof(payload) - 1);
        if (err != ESP_OK) {
            release_active(fd);
            return err;
        }
    }
    if (frame.type == HTTPD_WS_TYPE_CLOSE) {
        release_active(fd);
        broadcast_active();
        return ESP_OK;
    }
    if (frame.type != HTTPD_WS_TYPE_TEXT || !frame.final ||
        memchr(payload, '\0', frame.len) != NULL) {
        return ESP_OK;
    }
    payload[frame.len] = '\0';
    process_websocket_message(fd, payload);
    return ESP_OK;
}

void web_server_get_command(web_command_t *command)
{
    if (command == NULL) {
        return;
    }
    portENTER_CRITICAL(&state_mux);
    const int64_t now = esp_timer_get_time();
    const int64_t elapsed_us = last_command_us > 0 ? now - last_command_us : INT64_MAX;
    const int64_t age = elapsed_us == INT64_MAX ? INT64_MAX : elapsed_us / 1000;
    command->age_ms = age >= UINT32_MAX ? UINT32_MAX : (uint32_t)age;
    command->connected = active_fd >= 0 &&
                         elapsed_us <= WEB_COMMAND_TIMEOUT_MS * 1000LL;
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
    portENTER_CRITICAL(&state_mux);
    const bool dirty = active_dirty;
    const quadmd_telemetry_t t = telemetry_snapshot;
    portEXIT_CRITICAL(&state_mux);
    if (dirty) {
        broadcast_active();
    }
    wifi_status_t wifi = { .rssi = -127 };
    (void)wifi_sta_get_status(&wifi);
    char json[448];
    const int len = snprintf(json, sizeof(json),
        "{\"type\":\"telemetry\",\"valid\":%s,\"battery\":%.3f,"
        "\"rpm\":[%.1f,%.1f,%.1f,%.1f],\"cmd\":[%d,%d,%d,%d],"
        "\"comm\":%s,\"watchdog\":%s,\"fault\":%u,\"sequence\":%lu,"
        "\"wifi_connected\":%s,\"rssi\":%d}",
        t.valid ? "true" : "false", t.battery_voltage,
        t.rpm[0], t.rpm[1], t.rpm[2], t.rpm[3],
        t.motor_command[0], t.motor_command[1], t.motor_command[2], t.motor_command[3],
        t.communication_ok ? "true" : "false", t.watchdog_ok ? "true" : "false",
        (unsigned int)t.fault_status, (unsigned long)t.last_command_sequence,
        wifi.connected ? "true" : "false", wifi.rssi);
    int clients[WEB_MAX_CLIENTS];
    size_t count = WEB_MAX_CLIENTS;
    if (len > 0 && len < (int)sizeof(json)) {
        ESP_LOGI(TAG, "Telemetry JSON: %s", json);
    }
    if (len > 0 && len < (int)sizeof(json) &&
        httpd_get_client_list(server, &count, clients) == ESP_OK) {
        int websocket_clients[WEB_MAX_CLIENTS];
        size_t websocket_count = 0;
        for (size_t i = 0; i < count; ++i) {
            if (httpd_ws_get_fd_info(server, clients[i]) == HTTPD_WS_CLIENT_WEBSOCKET) {
                websocket_clients[websocket_count++] = clients[i];
            }
        }
        if (websocket_count > 0) {
            ESP_LOGI(TAG, "Broadcast telemetry to %d clients",
                     (int)websocket_count);
            for (size_t i = 0; i < websocket_count; ++i) {
                (void)send_text(websocket_clients[i], json);
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
        const bool enqueue = !broadcast_pending;
        if (enqueue) {
            broadcast_pending = true;
        }
        portEXIT_CRITICAL(&state_mux);
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
    /* Keep persistent WebSocket sessions predictable under connection pressure. */
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
    const httpd_uri_t style_uri = {
        .uri = "/style.css", .method = HTTP_GET, .handler = style_handler,
    };
    const httpd_uri_t app_uri = {
        .uri = "/app.js", .method = HTTP_GET, .handler = app_handler,
    };
    err = httpd_register_uri_handler(server, &root_uri);
    if (err == ESP_OK) {
        err = httpd_register_uri_handler(server, &style_uri);
    }
    if (err == ESP_OK) {
        err = httpd_register_uri_handler(server, &app_uri);
    }
    if (err == ESP_OK) {
        err = httpd_register_uri_handler(server, &websocket_uri);
    }
    if (err == ESP_OK && xTaskCreatePinnedToCore(telemetry_web_task,
            "web_telemetry", 4096, NULL, 4, NULL, 0) != pdPASS) {
        err = ESP_ERR_NO_MEM;
    }
    if (err != ESP_OK) {
        httpd_stop(server);
        server = NULL;
        return err;
    }
    ESP_LOGI(TAG, "HTTP/WebSocket started: %d clients", WEB_MAX_CLIENTS);
    return ESP_OK;
}
