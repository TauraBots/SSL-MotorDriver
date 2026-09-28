#pragma once

/* Serial timing profiles are independent from the robot mechanical profile. */
#define TAURA_SERIAL_PROFILE_NORMAL 0
#define TAURA_SERIAL_PROFILE_VALIDATION_120 3

#ifndef TAURA_SERIAL_PROFILE
#define TAURA_SERIAL_PROFILE TAURA_SERIAL_PROFILE_NORMAL
#endif

#if TAURA_SERIAL_PROFILE == TAURA_SERIAL_PROFILE_NORMAL

#define SERIAL_TELEMETRY_MIN_INTERVAL_MS 100U
#define SERIAL_TELEMETRY_TURNAROUND_MS 3U

#elif TAURA_SERIAL_PROFILE == TAURA_SERIAL_PROFILE_VALIDATION_120

/* 120 Hz is ~8.33 ms; allow margin for 1 ms turnaround and HAL tick quantization.
 * The E0 requests from the ESP32 still limit telemetry to 120 Hz. */
#define SERIAL_TELEMETRY_MIN_INTERVAL_MS 6U
#define SERIAL_TELEMETRY_TURNAROUND_MS 1U

#else
#error "Unsupported TAURA_SERIAL_PROFILE"
#endif
