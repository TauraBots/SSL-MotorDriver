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

    /* Telemetry broadcast remains multi-client. */
    connect_peer(12);
    quadmd_telemetry_t telemetry = {
        .valid=true, .battery_voltage=11.835f, .rpm={1,2,3,4},
        .communication_ok=true, .watchdog_ok=true, .fault_status=0,
        .last_command_sequence=42,
    };
    web_server_update_telemetry(&telemetry);
    peers[11].used=peers[12].used=0;
    message(11,"TAKE_CONTROL");
    broadcast_work(NULL);
    assert(peers[11].used>0 && peers[12].used>0);
    assert(wire_contains(11, "\"type\":\"telemetry\""));
    assert(wire_contains(12, "\"type\":\"telemetry\""));
    assert(wire_contains(11, "\"battery\":11.835"));
    assert(wire_contains(12, "\"rssi\":-61"));
    assert(active_fd==11); /* The observer receives telemetry without taking control. */
    assert(critical_depth==0);
    puts("PASS: active_fd takeover/release, global stop, watchdog, TX isolation, telemetry, disconnect");
    return 0;
}
