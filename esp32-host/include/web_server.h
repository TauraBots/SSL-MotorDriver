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

    /* True only while the exclusive controller has a fresh valid command. */
    bool connected;

    uint32_t age_ms;

} web_command_t;

// ============================================================

/* WS: CLAIM_CONTROL, RELEASE_CONTROL, HEARTBEAT, CMD,vx,vy,omega,kick,brake.
 * All clients receive telemetry and personalized type=control status. */
esp_err_t web_server_start(void);

// ============================================================

/* Atomic snapshot; consumes kick once. Timeout/release/disconnect brake safely. */
void web_server_get_command(
    web_command_t *command
);

// ============================================================

void web_server_update_telemetry(
    const quadmd_telemetry_t *telemetry
);