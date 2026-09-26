#pragma once

// ============================================================
// PERFIS
// ============================================================

#define TAURA_PROFILE_SAFE_9600  0
#define TAURA_PROFILE_HIGH_RATE  1
#define TAURA_PROFILE_CUSTOM     2

// ============================================================
// PERFIL ATUAL
// ============================================================

#define TAURA_PROFILE TAURA_PROFILE_SAFE_9600

// ============================================================
// ROBÔ
// ============================================================

#define TAURA_ROBOT_ID 'A'

// ============================================================
// UART
// ============================================================

#define QUADMD_UART_RX_GPIO 16
#define QUADMD_UART_TX_GPIO 17

// ============================================================
// SAFE 9600
// ============================================================

#if TAURA_PROFILE == TAURA_PROFILE_SAFE_9600

#define QUADMD_UART_BAUD 9600
#define QUADMD_COMMAND_HZ 20
#define QUADMD_SPLIT_TELEMETRY 0
#define QUADMD_TELEMETRY_FULL_HZ 5
#define WEB_TELEMETRY_HZ 20

// ============================================================
// HIGH RATE
// ============================================================

#elif TAURA_PROFILE == TAURA_PROFILE_HIGH_RATE

#define QUADMD_UART_BAUD 921600
#define QUADMD_COMMAND_HZ 50
#define QUADMD_SPLIT_TELEMETRY 1
#define QUADMD_TELEMETRY_FAST_HZ 125
#define QUADMD_TELEMETRY_FULL_HZ 10
#define WEB_TELEMETRY_HZ 60

// ============================================================
// CUSTOM
// ============================================================

#elif TAURA_PROFILE == TAURA_PROFILE_CUSTOM

#define QUADMD_UART_BAUD 921600
#define QUADMD_COMMAND_HZ 100
#define QUADMD_SPLIT_TELEMETRY 1
#define QUADMD_TELEMETRY_FAST_HZ 200
#define QUADMD_TELEMETRY_FULL_HZ 20
#define WEB_TELEMETRY_HZ 60

#else

#error "TAURA_PROFILE invalido"

#endif

// ============================================================
// WATCHDOG WEB
// ============================================================

#define WEB_COMMAND_TIMEOUT_MS 250

// Controller ownership is independent of the motion watchdog.
#define CONTROLLER_LEASE_TIMEOUT_MS 1500
#define CONTROLLER_HEARTBEAT_MS 100

// Allow Wi-Fi retransmissions without relaxing the movement watchdog above.
#define WEB_SOCKET_SEND_TIMEOUT_MS 250

// ============================================================
// UTIL
// ============================================================

#define HZ_TO_US(hz) (1000000LL / (hz))
