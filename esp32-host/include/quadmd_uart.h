#pragma once

#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

#include "app_config.h"

// ============================================================

esp_err_t quadmd_uart_init(void);

// ============================================================

int quadmd_uart_write(
    const uint8_t *data,
    size_t length
);

// ============================================================

int quadmd_uart_read(
    uint8_t *data,
    size_t length,
    uint32_t timeout_ms
);

// ============================================================

uint32_t quadmd_uart_get_error_count(void);
