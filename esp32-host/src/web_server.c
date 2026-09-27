#include "web_server.h"

#include <inttypes.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
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
static int64_t telemetry_rx_time_us;
static bool active_dirty = true;
static bool broadcast_pending;
static web_server_stats_t web_stats;
static esp_timer_handle_t telemetry_timer;

typedef struct {
    float vx;
    float vy;
    float omega;
    uint8_t kick_power;
    bool brake;
    uint32_t sequence;
    int64_t esp_tx_time_us;
} last_sent_command_t;
static last_sent_command_t last_sent_command = { .brake = true };
/* Immutable after startup; all WebSocket sends run on the HTTP task. */
static httpd_handle_t server;

/* HTTP-task-owned list used only to send one diagnostic frame per session. */
typedef struct {
    bool active;
    int fd;
} web_test_client_t;
static web_test_client_t test_clients[WEB_MAX_CLIENTS];

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

static void session_close(httpd_handle_t handle, int fd)
{
    (void)handle;
    release_active(fd);
    for (size_t i = 0; i < WEB_MAX_CLIENTS; ++i) {
        if (test_clients[i].active && test_clients[i].fd == fd) {
            test_clients[i].active = false;
            break;
        }
    }
    close(fd); /* HTTPD does not close sockets when close_fn is installed. */
}

static bool send_text(int fd, const char *text)
{
    const httpd_ws_client_info_t info = httpd_ws_get_fd_info(server, fd);
    if (info != HTTPD_WS_CLIENT_WEBSOCKET) {
        ESP_LOGW(TAG, "WS TX ignorado fd=%d info=%d", fd, (int)info);
        release_active(fd);
        return false;
    }
    const size_t len = strlen(text);
    httpd_ws_frame_t frame = {
        .type = HTTPD_WS_TYPE_TEXT,
        .payload = (uint8_t *)text,
        .len = len,
    };
#if WEB_TELEMETRY_DEBUG
    ESP_LOGI(TAG, "WS TX fd=%d payload=%s", fd, text);
#endif
    const esp_err_t err = httpd_ws_send_frame_async(server, fd, &frame);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "WS TX falhou fd=%d err=0x%x", fd, (unsigned int)err);
        return false;
    }
    return true;
}

/* Returns true exactly once for each descriptor/session discovered by HTTPD. */
static bool register_test_client(int fd)
{
    web_test_client_t *free_slot = NULL;
    for (size_t i = 0; i < WEB_MAX_CLIENTS; ++i) {
        if (test_clients[i].active && test_clients[i].fd == fd) {
            return false;
        }
        if (!test_clients[i].active && free_slot == NULL) {
            free_slot = &test_clients[i];
        }
    }
    if (free_slot == NULL) {
        ESP_LOGW(TAG, "Sem slot para registrar teste WS fd=%d", fd);
        return false;
    }
    *free_slot = (web_test_client_t){ .active = true, .fd = fd };
    ESP_LOGI(TAG, "WS conectado fd=%d", fd);
    return true;
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
    /* ESP-IDF 6 performs the HTTP upgrade before installing this handler. Every
     * invocation here is therefore an actual WebSocket frame, not the GET. */
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

void web_server_update_telemetry(const quadmd_telemetry_t *telemetry,
                                 int64_t esp_rx_time_us)
{
    if (telemetry == NULL) {
        return;
    }
    portENTER_CRITICAL(&state_mux);
    telemetry_snapshot = *telemetry;
    telemetry_rx_time_us = esp_rx_time_us;
    portEXIT_CRITICAL(&state_mux);
}

void web_server_update_sent_command(const web_command_t *command,
                                    uint32_t sequence,
                                    int64_t esp_tx_time_us)
{
    if (command == NULL) {
        return;
    }
    portENTER_CRITICAL(&state_mux);
    last_sent_command = (last_sent_command_t){
        .vx = command->vx,
        .vy = command->vy,
        .omega = command->omega,
        .kick_power = command->kick_power,
        .brake = command->brake,
        .sequence = sequence,
        .esp_tx_time_us = esp_tx_time_us,
    };
    portEXIT_CRITICAL(&state_mux);
}

void web_server_get_stats(web_server_stats_t *stats)
{
    if (stats == NULL) {
        return;
    }
    portENTER_CRITICAL(&state_mux);
    *stats = web_stats;
    portEXIT_CRITICAL(&state_mux);
}

/* Executed by HTTPD, serialized with handlers, session cleanup and other sends. */
static void broadcast_work(void *arg)
{
    (void)arg;
    portENTER_CRITICAL(&state_mux);
    const bool dirty = active_dirty;
    const quadmd_telemetry_t t = telemetry_snapshot;
    const int64_t rx_time_us = telemetry_rx_time_us;
    const last_sent_command_t command = last_sent_command;
    portEXIT_CRITICAL(&state_mux);
    if (dirty) {
        broadcast_active();
    }
    wifi_status_t wifi = { .rssi = -127 };
    (void)wifi_sta_get_status(&wifi);
    char json[768];
    const int len = snprintf(json, sizeof(json),
        "{\"type\":\"telemetry\",\"valid\":%s,"
        "\"esp_rx_time_us\":%" PRId64 ",\"quadmd_time_ms\":%lu,"
        "\"request_sequence\":%u,\"last_command_sequence\":%lu,"
        "\"command_time_us\":%" PRId64 ",\"command_sequence\":%lu,"
        "\"command\":{\"vx\":%.3f,\"vy\":%.3f,\"omega\":%.3f,"
        "\"kick\":%u,\"brake\":%s},\"battery\":%.3f,"
        "\"rpm\":[%.1f,%.1f,%.1f,%.1f],"
        "\"motor_cmd\":[%d,%d,%d,%d],\"cmd\":[%d,%d,%d,%d],"
        "\"comm\":%s,\"watchdog\":%s,\"fault\":%u,\"sequence\":%lu,"
        "\"wifi_connected\":%s,\"rssi\":%d}",
        t.valid ? "true" : "false", rx_time_us, (unsigned long)t.time_ms,
        (unsigned int)t.request_sequence, (unsigned long)t.last_command_sequence,
        command.esp_tx_time_us, (unsigned long)command.sequence,
        command.vx, command.vy, command.omega,
        (unsigned int)command.kick_power, command.brake ? "true" : "false",
        t.battery_voltage,
        t.rpm[0], t.rpm[1], t.rpm[2], t.rpm[3],
        t.motor_command[0], t.motor_command[1], t.motor_command[2], t.motor_command[3],
        t.motor_command[0], t.motor_command[1], t.motor_command[2], t.motor_command[3],
        t.communication_ok ? "true" : "false", t.watchdog_ok ? "true" : "false",
        (unsigned int)t.fault_status, (unsigned long)t.last_command_sequence,
        wifi.connected ? "true" : "false", wifi.rssi);
    int clients[WEB_MAX_CLIENTS];
    size_t count = WEB_MAX_CLIENTS;
#if WEB_TELEMETRY_DEBUG
    if (len > 0 && len < (int)sizeof(json)) {
        ESP_LOGI(TAG, "Telemetry JSON: %s", json);
    }
#endif
    const bool json_valid = len > 0 && len < (int)sizeof(json);
    const esp_err_t list_err = json_valid
        ? httpd_get_client_list(server, &count, clients)
        : ESP_FAIL;
    if (json_valid && list_err == ESP_OK) {
        int websocket_clients[WEB_MAX_CLIENTS];
        size_t websocket_count = 0;
        for (size_t i = 0; i < count; ++i) {
            if (httpd_ws_get_fd_info(server, clients[i]) == HTTPD_WS_CLIENT_WEBSOCKET) {
                const int fd = clients[i];
                websocket_clients[websocket_count++] = fd;
                if (register_test_client(fd)) {
                    (void)send_text(fd, "{\"type\":\"test\",\"value\":123}");
                    portENTER_CRITICAL(&state_mux);
                    active_dirty = true;
                    portEXIT_CRITICAL(&state_mux);
                }
            }
        }
        if (websocket_count > 0) {
#if WEB_TELEMETRY_DEBUG
            ESP_LOGI(TAG, "Broadcast telemetry to %d clients",
                     (int)websocket_count);
#endif
            for (size_t i = 0; i < websocket_count; ++i) {
                const bool sent = send_text(websocket_clients[i], json);
                portENTER_CRITICAL(&state_mux);
                if (sent) {
                    web_stats.telemetry_frames++;
                } else {
                    web_stats.telemetry_dropped++;
                }
                portEXIT_CRITICAL(&state_mux);
            }
        }
    } else {
        portENTER_CRITICAL(&state_mux);
        web_stats.telemetry_dropped++;
        portEXIT_CRITICAL(&state_mux);
    }
    portENTER_CRITICAL(&state_mux);
    broadcast_pending = false;
    portEXIT_CRITICAL(&state_mux);
}

static void telemetry_timer_callback(void *arg)
{
    (void)arg;
    portENTER_CRITICAL(&state_mux);
    const bool enqueue = !broadcast_pending;
    if (enqueue) {
        broadcast_pending = true;
    } else {
        web_stats.telemetry_dropped++;
    }
    portEXIT_CRITICAL(&state_mux);

    /* At most one pending broadcast: telemetry is best effort. */
    if (enqueue && httpd_queue_work(server, broadcast_work, NULL) != ESP_OK) {
        portENTER_CRITICAL(&state_mux);
        broadcast_pending = false;
        web_stats.telemetry_dropped++;
        portEXIT_CRITICAL(&state_mux);
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
    if (err == ESP_OK) {
        const esp_timer_create_args_t timer_args = {
            .callback = telemetry_timer_callback,
            .name = "web_telemetry",
        };
        err = esp_timer_create(&timer_args, &telemetry_timer);
    }
    if (err == ESP_OK) {
        err = esp_timer_start_periodic(telemetry_timer,
                                       HZ_TO_US(WEB_TELEMETRY_HZ));
    }
    if (err != ESP_OK) {
        if (telemetry_timer != NULL) {
            (void)esp_timer_stop(telemetry_timer);
            (void)esp_timer_delete(telemetry_timer);
            telemetry_timer = NULL;
        }
        httpd_stop(server);
        server = NULL;
        return err;
    }
    ESP_LOGI(TAG, "HTTP/WebSocket started: %d clients", WEB_MAX_CLIENTS);
    return ESP_OK;
}
