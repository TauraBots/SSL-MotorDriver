#include "quadmd_uart.h"

#include "driver/uart.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// ============================================================

#define QUADMD_UART UART_NUM_2

// ============================================================

#define QUADMD_UART_RX_BUFFER_SIZE 2048
#define QUADMD_UART_TX_BUFFER_SIZE 2048

// ============================================================

static uint32_t uart_error_count;
static portMUX_TYPE uart_stats_mux = portMUX_INITIALIZER_UNLOCKED;

// ============================================================

esp_err_t quadmd_uart_init(void)
{
    const uart_config_t config =
    {
        .baud_rate =
            QUADMD_UART_BAUD,

        .data_bits =
            UART_DATA_8_BITS,

        .parity =
            UART_PARITY_DISABLE,

        .stop_bits =
            UART_STOP_BITS_1,

        .flow_ctrl =
            UART_HW_FLOWCTRL_DISABLE,

        .source_clk =
            UART_SCLK_DEFAULT
    };

    // ========================================================
    // Parâmetros
    // ========================================================

    ESP_ERROR_CHECK(
        uart_param_config(
            QUADMD_UART,
            &config
        )
    );

    // ========================================================
    // GPIO
    // ========================================================

    ESP_ERROR_CHECK(
        uart_set_pin(
            QUADMD_UART,

            QUADMD_UART_TX_GPIO,
            QUADMD_UART_RX_GPIO,

            UART_PIN_NO_CHANGE,
            UART_PIN_NO_CHANGE
        )
    );

    // ========================================================
    // Driver
    // ========================================================

    ESP_ERROR_CHECK(
        uart_driver_install(
            QUADMD_UART,

            QUADMD_UART_RX_BUFFER_SIZE,
            QUADMD_UART_TX_BUFFER_SIZE,

            0,
            NULL,
            0
        )
    );

    // ========================================================
    // Limpa lixo inicial
    // ========================================================

    ESP_ERROR_CHECK(
        uart_flush_input(
            QUADMD_UART
        )
    );

    return ESP_OK;
}

// ============================================================

int quadmd_uart_write(
    const uint8_t *data,
    size_t length)
{
    if (
        data == NULL ||
        length == 0
    )
    {
        return 0;
    }

    const int written = uart_write_bytes(
        QUADMD_UART,
        data,
        length
    );
    if (written != (int)length)
    {
        portENTER_CRITICAL(&uart_stats_mux);
        uart_error_count++;
        portEXIT_CRITICAL(&uart_stats_mux);
    }
    return written;
}

// ============================================================

int quadmd_uart_read(
    uint8_t *data,
    size_t length,
    uint32_t timeout_ms)
{
    if (
        data == NULL ||
        length == 0
    )
    {
        return 0;
    }

    const int received = uart_read_bytes(
        QUADMD_UART,
        data,
        length,
        pdMS_TO_TICKS(
            timeout_ms
        )
    );
    if (received < 0)
    {
        portENTER_CRITICAL(&uart_stats_mux);
        uart_error_count++;
        portEXIT_CRITICAL(&uart_stats_mux);
    }
    return received;
}

// ============================================================

uint32_t quadmd_uart_get_error_count(void)
{
    portENTER_CRITICAL(&uart_stats_mux);
    const uint32_t errors = uart_error_count;
    portEXIT_CRITICAL(&uart_stats_mux);
    return errors;
}
