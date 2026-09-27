#pragma once

// ============================================================
// PERFIS
// ============================================================

#define TAURA_PROFILE_SAFE_9600  0
#define TAURA_PROFILE_HIGH_RATE  1
#define TAURA_PROFILE_CUSTOM     2
#define TAURA_PROFILE_VALIDATION_120 3

// ============================================================
// PERFIL ATUAL
// ============================================================

#ifndef TAURA_PROFILE
#define TAURA_PROFILE TAURA_PROFILE_SAFE_9600
#endif

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
#define QUADMD_TELEMETRY_FAST_HZ 100
#define QUADMD_TELEMETRY_FULL_HZ 10
#define WEB_TELEMETRY_HZ 60

// ============================================================
// VALIDATION 120
// Perfil experimental para ensaios de validacao cinematica.
// ============================================================

#elif TAURA_PROFILE == TAURA_PROFILE_VALIDATION_120

#define QUADMD_UART_BAUD 921600
#define QUADMD_COMMAND_HZ 120
#define QUADMD_SPLIT_TELEMETRY 1
#define QUADMD_TELEMETRY_FAST_HZ 120
#define QUADMD_TELEMETRY_FULL_HZ 10
#define WEB_TELEMETRY_HZ 120

#else

#error "TAURA_PROFILE invalido"

#endif

// ============================================================
// WATCHDOG WEB
// ============================================================

#define WEB_COMMAND_TIMEOUT_MS 250

// Allow Wi-Fi retransmissions without relaxing the movement watchdog above.
#define WEB_SOCKET_SEND_TIMEOUT_MS 250

// TX POWER is the ESP32 transmit level; RSSI is the AP signal received by it.
#define WIFI_TX_POWER_DBM 20
#define WIFI_TX_POWER_QUARTER_DBM (WIFI_TX_POWER_DBM * 4)
#define WIFI_MONITOR_PERIOD_MS 1000

#if WIFI_TX_POWER_QUARTER_DBM < 8 || WIFI_TX_POWER_QUARTER_DBM > 84
#error "WIFI_TX_POWER_DBM fora do intervalo aceito pela API ESP-IDF"
#endif

// ============================================================
// UTIL
// ============================================================

#define HZ_TO_US(hz) (1000000LL / (hz))

// Logs por frame introduzem jitter significativo no perfil de 120 Hz.
#ifndef WEB_TELEMETRY_DEBUG
#define WEB_TELEMETRY_DEBUG 0
#endif
