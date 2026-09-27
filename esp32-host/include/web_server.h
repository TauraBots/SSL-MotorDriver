#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

#include "quadmd_protocol.h"

// ============================================================
// COMANDO WEB
// ============================================================

typedef struct
{
    float vx;
    float vy;
    float omega;

    uint8_t kick_power;

    bool brake;

    /* True only while the active client has a fresh valid command. */
    bool connected;

    uint32_t age_ms;

} web_command_t;

typedef struct
{
    uint32_t telemetry_frames;
    uint32_t telemetry_dropped;
} web_server_stats_t;

// ============================================================

/* WS: TAKE_CONTROL, RELEASE_CONTROL, EMERGENCY_STOP and
 * CMD,vx,vy,omega,kick,brake. All clients receive telemetry and a
 * personalized type=active status. */
esp_err_t web_server_start(void);

// ============================================================

/* Atomic snapshot; consumes kick once. Timeout/release/disconnect brake safely. */
void web_server_get_command(
    web_command_t *command
);

// ============================================================

void web_server_update_telemetry(
    const quadmd_telemetry_t *telemetry,
    int64_t esp_rx_time_us
);

// ============================================================

void web_server_update_sent_command(
    const web_command_t *command,
    uint32_t sequence,
    int64_t esp_tx_time_us
);

void web_server_get_stats(
    web_server_stats_t *stats
);
