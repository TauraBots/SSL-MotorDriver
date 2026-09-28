/* Appended after the real firmware source by run_web_server_host_tests.py. */
static void message(int fd, const char *input)
{
    httpd_req_t req = {.fd=fd, .input=input};
    assert(websocket_handler(&req)==ESP_OK);
}
static void connect_peer(int fd)
{
    peers[fd].info=HTTPD_WS_CLIENT_WEBSOCKET;
}
static void assert_safe(void)
{
    web_command_t c;
    web_server_get_command(&c);
    assert(c.vx==0 && c.vy==0 && c.omega==0 && c.kick_power==0 && c.brake);
}
static bool wire_contains(int fd, const char *text)
{
    const size_t length = strlen(text);
    for (size_t i=0; i+length<=peers[fd].used; ++i) {
        if (memcmp(peers[fd].wire+i, text, length)==0) return true;
    }
    return false;
}
static size_t wire_count(int fd, const char *text)
{
    const size_t length = strlen(text);
    size_t count = 0;
    for (size_t i=0; i+length<=peers[fd].used; ++i) {
        if (memcmp(peers[fd].wire+i, text, length)==0) ++count;
    }
    return count;
}
static const unsigned char *wire_find_after(int fd, const char *text,
                                             const unsigned char *after)
{
    const size_t length = strlen(text);
    size_t start = after == NULL ? 0 : (size_t)(after-peers[fd].wire)+1;
    for (size_t i=start; i+length<=peers[fd].used; ++i) {
        if (memcmp(peers[fd].wire+i, text, length)==0) return peers[fd].wire+i;
    }
    return NULL;
}
static quadmd_telemetry_t race_telemetry;
static void enqueue_during_pending_clear(void)
{
    web_server_update_telemetry(&race_telemetry, 900000003);
}
int main(void)
{
    assert(web_server_start()==ESP_OK);
    connect_peer(10); connect_peer(11);
    assert(active_fd==-1); /* Connecting only observes telemetry. */
    broadcast_work(NULL);
    assert(wire_contains(10, "{\"type\":\"test\",\"value\":123}"));
    assert(wire_contains(11, "{\"type\":\"test\",\"value\":123}"));

    message(10,"TAKE_CONTROL");
    assert(active_fd==10);
    message(10,"CMD,1,0,0,50,0");
    message(10,"CMD,1,0,0,0,0");
    web_command_t c;
    web_server_get_command(&c);
    assert(c.connected && c.vx==1 && c.kick_power==50);
    web_server_get_command(&c); assert(c.kick_power==0);

    message(11,"CMD,2,1,1,80,0");
    web_server_get_command(&c);
    assert(c.vx==1 && c.vy==0 && c.omega==0 && c.kick_power==0);

    message(11,"TAKE_CONTROL");
    assert(active_fd==11); assert_safe(); /* Safe stop precedes takeover. */
    message(11,"CMD,2,1,1,80,0");
    web_server_get_command(&c);
    assert(c.connected && c.vx==2 && c.vy==1 && c.omega==1 && c.kick_power==80);
    message(10,"CMD,-1,-1,-1,10,0");
    web_server_get_command(&c); assert(c.vx==2 && c.vy==1 && c.omega==1);

    message(10,"RELEASE_CONTROL"); assert(active_fd==11);
    message(11,"RELEASE_CONTROL"); assert(active_fd==-1); assert_safe();

    message(10,"TAKE_CONTROL");
    message(10,"CMD,1,0,0,0,0");
    mock_now+=WEB_COMMAND_TIMEOUT_MS*1000LL+1;
    assert_safe();
    assert(active_fd==10); /* Motion timeout does not change the active client. */

    message(11,"EMERGENCY_STOP");
    assert(active_fd==-1); assert_safe();

    /* Native HTTPD WebSocket TX emits a complete text frame and isolates errors. */
    message(10,"TAKE_CONTROL");
    peers[10].used=peers[11].used=0;
    assert(send_text(10,"first"));
    const unsigned char expected[]={0x81,5,'f','i','r','s','t'};
    assert(peers[10].used==sizeof(expected));
    assert(memcmp(peers[10].wire,expected,sizeof(expected))==0);
    peers[10].blocked=1;
    assert(!send_text(10,"blocked"));
    assert(send_text(11,"observer")); assert(peers[11].used>0);
    peers[10].blocked=0;

    /* Session cleanup releases control and descriptor reuse is clean. */
    session_close(server,10);
    assert(active_fd==-1); assert_safe();
    connect_peer(10);
    message(10,"TAKE_CONTROL");
    session_close(server,10);
    assert(active_fd==-1); assert_safe();

    /* Event-driven telemetry remains multi-client and preserves every E1. */
    connect_peer(12);
    quadmd_telemetry_t telemetry = {
        .valid=true, .battery_voltage=11.835f, .rpm={1,2,3,4},
        .motor_command={10,20,30,40}, .time_ms=654321,
        .request_sequence=77,
        .communication_ok=true, .watchdog_ok=true, .fault_status=0,
        .last_command_sequence=42,
    };
    web_command_t sent_command = {
        .vx=0.5f, .vy=0.0f, .omega=0.25f, .kick_power=7, .brake=false,
    };
    web_server_stats_t stats_before;
    web_server_get_stats(&stats_before);
    peers[11].used=peers[12].used=0;
    message(11,"TAKE_CONTROL");
    mock_defer_work=true;

    web_server_update_sent_command(&sent_command, 4890, 123450000);
    telemetry.request_sequence=100;
    web_server_update_telemetry(&telemetry, 123456789);

    sent_command.vx=0.75f;
    web_server_update_sent_command(&sent_command, 4891, 123450001);
    telemetry.request_sequence=101;
    web_server_update_telemetry(&telemetry, 123456790);

    sent_command.vx=1.0f;
    web_server_update_sent_command(&sent_command, 4892, 123450002);
    telemetry.request_sequence=102;
    web_server_update_telemetry(&telemetry, 123456791);

    assert(mock_work_requests==1);
    assert(uxQueueMessagesWaiting(telemetry_queue)==3);
    run_queued_work();
    assert(mock_pending_work==NULL);
    assert(peers[11].used>0 && peers[12].used>0);
    assert(wire_count(11, "{\"type\":\"telemetry\"")==3);
    assert(wire_count(12, "{\"type\":\"telemetry\"")==3);
    assert(wire_contains(11, "\"battery\":11.835"));
    assert(wire_contains(12, "\"rssi\":-61"));
    assert(wire_contains(11, "\"esp_rx_time_us\":123456789"));
    assert(wire_contains(11, "\"esp_rx_time_us\":123456790"));
    assert(wire_contains(11, "\"esp_rx_time_us\":123456791"));
    assert(wire_contains(11, "\"quadmd_time_ms\":654321"));
    assert(wire_contains(11, "\"request_sequence\":100"));
    assert(wire_contains(11, "\"request_sequence\":101"));
    assert(wire_contains(11, "\"request_sequence\":102"));
    assert(wire_contains(11, "\"command_time_us\":123450000"));
    assert(wire_contains(11, "\"command_sequence\":4890"));
    assert(wire_contains(11, "\"command_sequence\":4891"));
    assert(wire_contains(11, "\"command_sequence\":4892"));
    assert(wire_contains(11, "\"last_command_sequence\":42"));
    assert(wire_contains(11, "\"command\":{\"vx\":0.500,\"vy\":0.000,\"omega\":0.250,\"kick\":7,\"brake\":false}"));
    assert(wire_contains(11, "\"command\":{\"vx\":0.750"));
    assert(wire_contains(11, "\"command\":{\"vx\":1.000"));
    assert(wire_contains(11, "\"rpm\":[1.0,2.0,3.0,4.0]"));
    assert(wire_contains(11, "\"motor_cmd\":[10,20,30,40]"));
    assert(wire_contains(11, "\"cmd\":[10,20,30,40]"));
    const unsigned char *seq100=wire_find_after(11, "\"request_sequence\":100", NULL);
    const unsigned char *seq101=wire_find_after(11, "\"request_sequence\":101", seq100);
    const unsigned char *seq102=wire_find_after(11, "\"request_sequence\":102", seq101);
    assert(seq100!=NULL && seq101!=NULL && seq102!=NULL);
    assert(seq100<seq101 && seq101<seq102);
    web_server_stats_t stats;
    web_server_get_stats(&stats);
    assert(stats.telemetry_broadcasts==stats_before.telemetry_broadcasts+3);
    assert(stats.telemetry_client_frames==stats_before.telemetry_client_frames+6);
    assert(stats.telemetry_dropped==0);

    /* Empty worker calls never duplicate a telemetry frame. */
    const size_t telemetry_count_11=wire_count(11, "{\"type\":\"telemetry\"");
    const size_t telemetry_count_12=wire_count(12, "{\"type\":\"telemetry\"");
    broadcast_work(NULL);
    assert(wire_count(11, "{\"type\":\"telemetry\"")==telemetry_count_11);
    assert(wire_count(12, "{\"type\":\"telemetry\"")==telemetry_count_12);
    web_server_get_stats(&stats);
    assert(stats.telemetry_broadcasts==stats_before.telemetry_broadcasts+3);
    assert(stats.telemetry_client_frames==stats_before.telemetry_client_frames+6);

    /* Arrival during the worker's pending-clear window is rescheduled. */
    telemetry.request_sequence=200;
    web_server_update_telemetry(&telemetry, 900000002);
    race_telemetry=telemetry;
    race_telemetry.request_sequence=201;
    mock_queue_check_hook=enqueue_during_pending_clear;
    run_queued_work();
    assert(mock_pending_work!=NULL);
    run_queued_work();
    assert(mock_pending_work==NULL);
    assert(wire_contains(11, "\"request_sequence\":200"));
    assert(wire_contains(11, "\"request_sequence\":201"));

    /* Full queue is non-blocking DROP NEWEST; no-client draining is separate. */
    peers[11].info=HTTPD_WS_CLIENT_INVALID;
    peers[12].info=HTTPD_WS_CLIENT_INVALID;
    const uint32_t dropped_before=stats.telemetry_dropped;
    for (unsigned int i=0; i<WEB_TELEMETRY_QUEUE_DEPTH; ++i) {
        telemetry.request_sequence=(uint16_t)(300+i);
        web_server_update_telemetry(&telemetry, 910000000+i);
    }
    assert(uxQueueMessagesWaiting(telemetry_queue)==WEB_TELEMETRY_QUEUE_DEPTH);
    telemetry.request_sequence=999;
    web_server_update_telemetry(&telemetry, 920000000);
    web_server_get_stats(&stats);
    assert(stats.telemetry_dropped==dropped_before+1);
    while (mock_pending_work!=NULL) run_queued_work();
    assert(uxQueueMessagesWaiting(telemetry_queue)==0);
    web_server_get_stats(&stats);
    assert(stats.telemetry_no_client>=WEB_TELEMETRY_QUEUE_DEPTH);
    assert(stats.telemetry_dropped==dropped_before+1);

    assert(active_fd==11); /* The observer receives telemetry without taking control. */
    assert(critical_depth==0);
    puts("PASS: control, watchdog, TX isolation, FIFO telemetry, overflow, race, no-client");
    return 0;
}
