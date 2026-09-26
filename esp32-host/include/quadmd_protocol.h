#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// ============================================================
// SOF
// ============================================================

#define QUADMD_SOF0 0x55U
#define QUADMD_SOF1 0xAAU

// ============================================================
// TIPOS
// ============================================================

#define QUADMD_TYPE_VELOCITY           0xD0U

#define QUADMD_TYPE_TELEMETRY_REQUEST  0xE0U
#define QUADMD_TYPE_TELEMETRY_RESPONSE 0xE1U

#define QUADMD_TYPE_DISCOVERY_REQUEST  0xE2U
#define QUADMD_TYPE_DISCOVERY_RESPONSE 0xE3U

#define QUADMD_TYPE_CONFIG_DISCOVER        0xF0U
#define QUADMD_TYPE_CONFIG_SET_ID          0xF1U
#define QUADMD_TYPE_CONFIG_DISCOVER_RESP   0xF2U
#define QUADMD_TYPE_CONFIG_SET_ID_RESP     0xF3U
#define QUADMD_TYPE_CONFIG_SET_MOTION      0xF4U
#define QUADMD_TYPE_CONFIG_SET_MOTION_RESP 0xF5U

// ============================================================
// VERSÃO
// ============================================================

#define QUADMD_PROTOCOL_VERSION 1U

// ============================================================
// TELEMETRY FLAGS
// ============================================================

#define QUADMD_TELEMETRY_BASIC        0x01U
#define QUADMD_TELEMETRY_MOTORS       0x02U
#define QUADMD_TELEMETRY_BATTERY      0x04U
#define QUADMD_TELEMETRY_DIAGNOSTICS  0x08U

#define QUADMD_TELEMETRY_FULL 0x0FU

#define QUADMD_TELEMETRY_FAST \
    (QUADMD_TELEMETRY_BASIC | QUADMD_TELEMETRY_MOTORS)

// ============================================================
// TAMANHOS
// ============================================================

#define QUADMD_VELOCITY_PACKET_SIZE    19U
#define QUADMD_TELEMETRY_REQUEST_SIZE  10U
#define QUADMD_TELEMETRY_MAX_SIZE      51U

// ============================================================
// TELEMETRIA
// ============================================================

typedef struct
{
    bool valid;

    uint8_t robot_id;

    uint16_t request_sequence;

    uint8_t status;
    uint8_t fault_status;

    uint8_t flags;

    // ========================================================
    // BASIC
    // ========================================================

    uint32_t time_ms;

    uint8_t communication_ok;
    uint8_t brake;
    uint8_t kick_power;

    // ========================================================
    // MOTORS
    // ========================================================

    float rpm[4];

    int16_t motor_command[4];

    // ========================================================
    // BATTERY
    // ========================================================

    float battery_voltage;

    uint16_t battery_adc;

    // ========================================================
    // DIAGNOSTICS
    // ========================================================

    uint32_t crc_errors;

    uint32_t received_packets;

    uint8_t watchdog_ok;

    uint32_t last_command_sequence;

} quadmd_telemetry_t;

// ============================================================
// CRC
// ============================================================

uint16_t quadmd_crc16(
    const uint8_t *data,
    size_t length
);

// ============================================================
// D0
// ============================================================

size_t quadmd_encode_velocity(
    uint8_t *buffer,

    uint8_t robot_id,

    uint32_t sequence,

    float vx,
    float vy,
    float omega,

    uint8_t kick_power,

    bool brake
);

// ============================================================
// E0
// ============================================================

size_t quadmd_encode_telemetry_request(
    uint8_t *buffer,

    uint8_t robot_id,

    uint16_t request_sequence,

    uint8_t flags
);

// ============================================================
// E1
// ============================================================

size_t quadmd_telemetry_frame_size(
    uint8_t flags
);

bool quadmd_parse_telemetry(
    const uint8_t *frame,

    size_t length,

    uint8_t expected_robot_id,

    quadmd_telemetry_t *telemetry
);