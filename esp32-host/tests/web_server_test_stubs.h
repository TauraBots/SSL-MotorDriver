/* Minimal mocked IDF surface for compiling the actual web_server.c on a host.
 * Every network operation asserts that no critical section is held. */
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <errno.h>
#include <limits.h>
#include <stdarg.h>

static const unsigned char web_index_html[] = "<html>";
static const unsigned char web_style_css[] = "body{}";
static const unsigned char web_app_js[] = "'use strict';";
#define WEB_INDEX_HTML_LEN ((size_t)(sizeof(web_index_html) - 1U))
#define WEB_STYLE_CSS_LEN ((size_t)(sizeof(web_style_css) - 1U))
#define WEB_APP_JS_LEN ((size_t)(sizeof(web_app_js) - 1U))

typedef int esp_err_t;
#define ESP_OK 0
#define ESP_FAIL -1
#define ESP_ERR_INVALID_ARG -2
#define ESP_ERR_INVALID_SIZE -3
#define ESP_ERR_NO_MEM -4
#define ESP_ERR_INVALID_STATE -5
#define HTTPD_SOCK_ERR_FAIL -1
#define HTTP_GET 1
#define HTTPD_RESP_USE_STRLEN -1
#define MSG_DONTWAIT 1
#define SHUT_RDWR 2
#define HTTPD_WS_TYPE_TEXT 1
#define HTTPD_WS_TYPE_CLOSE 8
#define HTTPD_WS_TYPE_PONG 10
#define portMUX_INITIALIZER_UNLOCKED 0
#define pdPASS 1
#define pdMS_TO_TICKS(ms) (ms)
typedef int portMUX_TYPE;
typedef unsigned int TickType_t;
static int critical_depth;
#define portENTER_CRITICAL(m) ((void)(m), ++critical_depth)
#define portEXIT_CRITICAL(m) ((void)(m), --critical_depth)
static void mock_log(const char *tag, const char *format, ...) { (void)tag; (void)format; assert(!critical_depth); }
#define ESP_LOGI mock_log
#define ESP_LOGW mock_log
static int64_t mock_now = 100000;
static int64_t esp_timer_get_time(void) { return mock_now; }
typedef struct { bool connected; int rssi; uint8_t channel; } wifi_status_t;
static bool wifi_sta_get_status(wifi_status_t *status) {
    *status=(wifi_status_t){.connected=true,.rssi=-61,.channel=6}; return true;
}
static void vTaskDelay(TickType_t t) { (void)t; }
static int xTaskCreatePinnedToCore(void (*fn)(void *), const char *name, int stack, void *arg, int priority, void *handle, int core) {
    (void)fn; (void)name; (void)stack; (void)arg; (void)priority; (void)handle; (void)core; return pdPASS;
}
typedef void *httpd_handle_t;
typedef enum { HTTPD_WS_CLIENT_INVALID, HTTPD_WS_CLIENT_HTTP, HTTPD_WS_CLIENT_WEBSOCKET } httpd_ws_client_info_t;
typedef struct { int type; uint8_t *payload; size_t len; bool final; } httpd_ws_frame_t;
typedef struct { int method; int fd; const char *input; } httpd_req_t;
typedef struct { const char *uri; int method; esp_err_t (*handler)(httpd_req_t *); bool is_websocket; } httpd_uri_t;
typedef struct {
    int max_open_sockets, recv_wait_timeout, send_wait_timeout;
    bool lru_purge_enable, keep_alive_enable;
    int keep_alive_idle, keep_alive_interval, keep_alive_count;
    void (*close_fn)(httpd_handle_t, int);
} httpd_config_t;
#define HTTPD_DEFAULT_CONFIG() ((httpd_config_t){0})
typedef int (*send_fn_t)(httpd_handle_t, int, const char *, size_t, int);
static struct {
    httpd_ws_client_info_t info;
    send_fn_t sender;
    int blocked, fatal, writes, closes;
    size_t limit, used;
    unsigned char wire[8192];
} peers[32];
static int mock_send(int fd, const void *buf, size_t len, int flags) {
    assert(!critical_depth && flags == MSG_DONTWAIT);
    ++peers[fd].writes;
    if (peers[fd].fatal) { errno=peers[fd].fatal; return -1; }
    if (peers[fd].blocked) { errno=EAGAIN; return -1; }
    if (peers[fd].limit && len>peers[fd].limit) len=peers[fd].limit;
    assert(peers[fd].used+len <= sizeof(peers[fd].wire));
    memcpy(peers[fd].wire+peers[fd].used,buf,len); peers[fd].used+=len;
    return (int)len;
}
#define send mock_send
static int mock_shutdown(int fd, int how) { assert(!critical_depth); (void)how; peers[fd].info=HTTPD_WS_CLIENT_INVALID; ++peers[fd].closes; return 0; }
#define shutdown mock_shutdown
static int mock_close(int fd) { assert(!critical_depth); peers[fd].info=HTTPD_WS_CLIENT_INVALID; ++peers[fd].closes; return 0; }
#define close mock_close
static httpd_ws_client_info_t httpd_ws_get_fd_info(httpd_handle_t h, int fd) { (void)h; assert(!critical_depth); return peers[fd].info; }
static esp_err_t httpd_sess_set_send_override(httpd_handle_t h, int fd, send_fn_t fn) { (void)h; assert(!critical_depth); peers[fd].sender=fn; return ESP_OK; }
static esp_err_t httpd_ws_send_frame_async(httpd_handle_t h, int fd, httpd_ws_frame_t *frame) {
    assert(!critical_depth && frame->len<=UINT16_MAX);
    unsigned char header[4] = {(unsigned char)(0x80|frame->type), 0, 0, 0};
    size_t header_len=2;
    if (frame->len<126) {
        header[1]=(unsigned char)frame->len;
    } else {
        header[1]=126; header[2]=(unsigned char)(frame->len>>8);
        header[3]=(unsigned char)frame->len; header_len=4;
    }
    if (peers[fd].sender(h,fd,(const char *)header,header_len,0)<0) return ESP_FAIL;
    if (frame->len && peers[fd].sender(h,fd,(const char *)frame->payload,frame->len,0)<0) return ESP_FAIL;
    return ESP_OK;
}
static esp_err_t httpd_get_client_list(httpd_handle_t h, size_t *count, int *fds) {
    (void)h; assert(!critical_depth); size_t n=0;
    for(int fd=0;fd<32;++fd) if(peers[fd].info!=HTTPD_WS_CLIENT_INVALID) { assert(n<*count); fds[n++]=fd; }
    *count=n; return ESP_OK;
}
static esp_err_t httpd_resp_set_type(httpd_req_t *r, const char *t) { (void)r;(void)t;return ESP_OK; }
static esp_err_t httpd_resp_set_hdr(httpd_req_t *r, const char *k, const char *v) { (void)r;(void)k;(void)v;return ESP_OK; }
static esp_err_t httpd_resp_send(httpd_req_t *r, const char *s, int n) { (void)r;(void)s;(void)n;return ESP_OK; }
static int httpd_req_to_sockfd(httpd_req_t *r) { return r->fd; }
static esp_err_t httpd_ws_recv_frame(httpd_req_t *r, httpd_ws_frame_t *f, size_t max_len) {
    f->len=strlen(r->input); f->type=HTTPD_WS_TYPE_TEXT; f->final=true;
    if(max_len) { assert(f->len<=max_len);memcpy(f->payload,r->input,f->len); } return ESP_OK;
}
static esp_err_t httpd_start(httpd_handle_t *h, httpd_config_t *cfg) { (void)cfg; *h=(void *)1;return ESP_OK; }
static esp_err_t httpd_stop(httpd_handle_t h) { (void)h;return ESP_OK; }
static esp_err_t httpd_register_uri_handler(httpd_handle_t h,const httpd_uri_t *u) { (void)h;(void)u;return ESP_OK; }
static esp_err_t httpd_queue_work(httpd_handle_t h,void(*fn)(void *),void *arg) { (void)h;fn(arg);return ESP_OK; }
