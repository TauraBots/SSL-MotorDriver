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
#include <inttypes.h>
#include <limits.h>
#include <stdarg.h>
#include <stdlib.h>
#include <sys/types.h>

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
#define HTTPD_WS_TYPE_TEXT 1
#define HTTPD_WS_TYPE_CLOSE 8
#define HTTPD_WS_TYPE_PONG 10
#define portMUX_INITIALIZER_UNLOCKED 0
#define pdPASS 1
#define pdTRUE 1
#define pdFALSE 0
#define pdMS_TO_TICKS(ms) (ms)
typedef int portMUX_TYPE;
typedef unsigned int TickType_t;
typedef unsigned int UBaseType_t;
typedef struct {
    size_t capacity, item_size, head, count;
    unsigned char *storage;
} mock_queue_t;
typedef mock_queue_t *QueueHandle_t;
static QueueHandle_t xQueueCreate(UBaseType_t capacity, UBaseType_t item_size) {
    mock_queue_t *q=calloc(1,sizeof(*q));
    if(!q) return NULL;
    q->storage=calloc(capacity,item_size);
    if(!q->storage) { free(q); return NULL; }
    q->capacity=capacity; q->item_size=item_size; return q;
}
static int xQueueSend(QueueHandle_t q, const void *item, TickType_t wait) {
    assert(wait==0);
    if(q->count==q->capacity) return pdFALSE;
    const size_t tail=(q->head+q->count)%q->capacity;
    memcpy(q->storage+tail*q->item_size,item,q->item_size); ++q->count;
    return pdTRUE;
}
static int xQueueReceive(QueueHandle_t q, void *item, TickType_t wait) {
    assert(wait==0);
    if(q->count==0) return pdFALSE;
    memcpy(item,q->storage+q->head*q->item_size,q->item_size);
    q->head=(q->head+1)%q->capacity; --q->count; return pdTRUE;
}
static void (*mock_queue_check_hook)(void);
static UBaseType_t uxQueueMessagesWaiting(QueueHandle_t q) {
    if(mock_queue_check_hook) {
        void (*hook)(void)=mock_queue_check_hook; mock_queue_check_hook=NULL; hook();
    }
    return (UBaseType_t)q->count;
}
static void vQueueDelete(QueueHandle_t q) { free(q->storage); free(q); }
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
static struct {
    httpd_ws_client_info_t info;
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
static int mock_close(int fd) { assert(!critical_depth); peers[fd].info=HTTPD_WS_CLIENT_INVALID; ++peers[fd].closes; return 0; }
#define close mock_close
static httpd_ws_client_info_t httpd_ws_get_fd_info(httpd_handle_t h, int fd) { (void)h; assert(!critical_depth); return peers[fd].info; }
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
    (void)h;
    int sent=mock_send(fd,header,header_len,MSG_DONTWAIT);
    if (sent<0) return ESP_FAIL;
    if (frame->len) {
        sent=mock_send(fd,frame->payload,frame->len,MSG_DONTWAIT);
        if (sent<0) return ESP_FAIL;
    }
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
static bool mock_defer_work;
static void (*mock_pending_work)(void *);
static void *mock_pending_arg;
static unsigned int mock_work_requests;
static esp_err_t httpd_queue_work(httpd_handle_t h,void(*fn)(void *),void *arg) {
    (void)h; ++mock_work_requests;
    if(mock_defer_work) {
        assert(mock_pending_work==NULL);
        mock_pending_work=fn; mock_pending_arg=arg;
    } else {
        fn(arg);
    }
    return ESP_OK;
}
static void run_queued_work(void) {
    assert(mock_pending_work!=NULL);
    void (*fn)(void *)=mock_pending_work; void *arg=mock_pending_arg;
    mock_pending_work=NULL; mock_pending_arg=NULL; fn(arg);
}
