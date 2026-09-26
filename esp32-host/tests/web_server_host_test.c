/* Appended after the real firmware source by run_web_server_host_tests.py. */
static void message(int fd, const char *input)
{
    httpd_req_t req = {.fd=fd, .input=input};
    assert(websocket_handler(&req)==ESP_OK);
}
static void connect_peer(int fd)
{
    peers[fd].info=HTTPD_WS_CLIENT_WEBSOCKET;
    httpd_req_t req = {.fd=fd, .method=HTTP_GET};
    assert(websocket_handler(&req)==ESP_OK);
}
static void assert_safe(void)
{
    web_command_t c;
    web_server_get_command(&c);
    assert(c.vx==0 && c.vy==0 && c.omega==0 && c.kick_power==0 && c.brake);
}
int main(void)
{
    assert(web_server_start()==ESP_OK);
    assert(timer_period==CONTROLLER_LEASE_CHECK_PERIOD_US);
    connect_peer(10); connect_peer(11);
    assert(controller_fd==-1); /* Handshake never grants control. */
    message(10,"CLAIM_CONTROL"); message(11,"CLAIM_CONTROL");
    assert(controller_fd==10);
    peers[10].used=peers[11].used=0;
    peers[10].blocked=1;
    assert(send_text(10,"first")); /* Accepted into bounded queue despite EAGAIN. */
    assert(peers[10].closes==0 && controller_fd==10);
    assert(peers[10].info==HTTPD_WS_CLIENT_WEBSOCKET);
    assert(!send_text(10,"dropped"));
    assert(send_text(11,"spectator"));
    assert(peers[11].used>0); /* Slow controller cannot stop spectator telemetry. */
    mock_now+=100000;
    message(10,"HEARTBEAT");
    assert(controller_last_seen_us==mock_now && controller_fd==10);
    const int64_t owner_seen=controller_last_seen_us;
    message(11,"HEARTBEAT"); message(11,"CMD,1,1,1,50,0");
    assert(controller_last_seen_us==owner_seen);
    // Recover with partial writes, including an interleaved protocol PONG.
    peers[10].blocked=0; peers[10].limit=1;
    httpd_ws_frame_t pong={.type=HTTPD_WS_TYPE_PONG,.payload=(uint8_t *)"x",.len=1};
    assert(httpd_ws_send_frame_async(server,10,&pong)==ESP_OK);
    for(int i=0;i<20;++i) (void)flush_tx(find_tx(10));
    const unsigned char expected[]={0x81,5,'f','i','r','s','t',0x8a,1,'x'};
    assert(peers[10].used==sizeof(expected));
    assert(memcmp(peers[10].wire,expected,sizeof(expected))==0);
    assert(find_tx(10)->failures==0 && peers[10].closes==0);
    peers[10].limit=0;
    message(10,"CMD,1,0,0,50,0");
    message(10,"CMD,1,0,0,0,0");
    web_command_t c;
    web_server_get_command(&c); assert(c.kick_power==50 && c.vx==1);
    web_server_get_command(&c); assert(c.kick_power==0);
    mock_now+=300000; message(10,"HEARTBEAT");
    assert_safe(); assert(controller_fd==10); /* Heartbeat cannot refresh motion. */
    message(11,"RELEASE_CONTROL"); assert(controller_fd==10);
    control_dirty=false;
    mock_now+=CONTROLLER_LEASE_TIMEOUT_MS*1000LL;
    lease_timer_callback(NULL);
    assert(controller_fd==-1 && control_dirty); assert_safe();
    message(10,"HEARTBEAT"); assert(controller_fd==-1); /* No resurrection. */
    message(11,"CLAIM_CONTROL"); assert(controller_fd==11);
    // Real socket death does release ownership immediately.
    peers[11].fatal=ECONNRESET;
    (void)send_text(11,"dead");
    assert(peers[11].closes==1 && controller_fd==-1); assert_safe();
    session_close(server,11); assert(find_tx(11)==NULL);
    peers[11].fatal=0;
    connect_peer(11); assert(find_tx(11)->length==0); /* fd reuse is clean. */
    message(11,"CLAIM_CONTROL");
    peers[11].info=HTTPD_WS_CLIENT_INVALID;
    assert(!controller_is_valid() && controller_fd==-1);
    assert(critical_depth==0);
    puts("PASS: transient TX, partial frames/PONG ordering, peer isolation, fd reuse, lease, watchdog, heartbeat, kick");
    return 0;
}
