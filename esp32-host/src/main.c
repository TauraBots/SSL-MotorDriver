#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "esp_timer.h"

#include "app_config.h"

#include "quadmd_protocol.h"
#include "quadmd_uart.h"

#include "wifi_sta.h"
#include "mdns_service.h"
#include "web_server.h"

// ============================================================

static const char *TAG =
    "TAURA_HOST";

// ============================================================
// ROBOT
// ============================================================

#define ROBOT_ID \
    ((uint8_t)TAURA_ROBOT_ID)

// ============================================================
// SEQUÊNCIAS
// ============================================================

static uint32_t
    command_sequence = 0;

static uint16_t
    telemetry_sequence = 0;

// ============================================================
// RX
// ============================================================

#define RX_BUFFER_SIZE 256

static uint8_t
    rx_buffer[RX_BUFFER_SIZE];

static size_t
    rx_length = 0;

// ============================================================
// TELEMETRIA
// ============================================================

static quadmd_telemetry_t
    telemetry;

static portMUX_TYPE
    telemetry_mux =
        portMUX_INITIALIZER_UNLOCKED;

// ============================================================
// ESTATÍSTICAS
// ============================================================

static uint32_t
    telemetry_packets = 0;

static uint32_t
    telemetry_parser_errors = 0;

static uint32_t
    telemetry_rx_overflows = 0;

// ============================================================
// SEND VELOCITY
// ============================================================

static void send_velocity(void)
{
    uint8_t frame[
        QUADMD_VELOCITY_PACKET_SIZE
    ];

    web_command_t command;

    web_server_get_command(
        &command
    );

    command_sequence++;

    const size_t length =
        quadmd_encode_velocity(
            frame,

            ROBOT_ID,

            command_sequence,

            command.vx,
            command.vy,
            command.omega,

            command.kick_power,

            command.brake
        );

    if (length == 0)
    {
        return;
    }

    const int64_t tx_time_us =
        esp_timer_get_time();

    const int written = quadmd_uart_write(
        frame,
        length
    );

    if (written == (int)length)
    {
        web_server_update_sent_command(
            &command,
            command_sequence,
            tx_time_us
        );
    }
}

// ============================================================
// REQUEST TELEMETRY
// ============================================================

static void request_telemetry(
    uint8_t flags)
{
    uint8_t frame[
        QUADMD_TELEMETRY_REQUEST_SIZE
    ];

    telemetry_sequence++;

    const size_t length =
        quadmd_encode_telemetry_request(
            frame,

            ROBOT_ID,

            telemetry_sequence,

            flags
        );

    if (length == 0)
    {
        return;
    }

    quadmd_uart_write(
        frame,
        length
    );
}

// ============================================================
// REMOVE RX
// ============================================================

static void remove_rx_bytes(
    size_t count)
{
    if (count >= rx_length)
    {
        rx_length = 0;

        return;
    }

    memmove(
        rx_buffer,

        &rx_buffer[count],

        rx_length - count
    );

    rx_length -=
        count;
}

// ============================================================
// PROCESS RX
// ============================================================

static void process_rx(void)
{
    uint8_t temp[128];

    const int received =
        quadmd_uart_read(
            temp,
            sizeof(temp),
            0
        );

    // ========================================================
    // Copia para stream buffer
    // ========================================================

    if (received > 0)
    {
        size_t received_size =
            (size_t)received;

        size_t available =
            RX_BUFFER_SIZE -
            rx_length;

        /*
         * Em caso de overflow,
         * resetamos o parser.
         */
        if (received_size > available)
        {
            rx_length = 0;

            portENTER_CRITICAL(
                &telemetry_mux
            );

            telemetry_rx_overflows++;

            portEXIT_CRITICAL(
                &telemetry_mux
            );

            available =
                RX_BUFFER_SIZE;
        }

        if (received_size > available)
        {
            received_size =
                available;
        }

        memcpy(
            &rx_buffer[rx_length],
            temp,
            received_size
        );

        rx_length +=
            received_size;
    }

    // ========================================================
    // Parser
    // ========================================================

    while (rx_length >= 3U)
    {
        // ====================================================
        // Procura 55 AA
        // ====================================================

        if (
            rx_buffer[0] !=
                QUADMD_SOF0 ||

            rx_buffer[1] !=
                QUADMD_SOF1
        )
        {
            remove_rx_bytes(1U);

            continue;
        }

        // ====================================================
        // Tipo
        // ====================================================

        const uint8_t type =
            rx_buffer[2];

        /*
         * Por enquanto só interpretamos E1.
         *
         * Boot text e outros pacotes são descartados
         * até encontrarmos o próximo 55 AA.
         */
        if (
            type !=
            QUADMD_TYPE_TELEMETRY_RESPONSE
        )
        {
            remove_rx_bytes(1U);

            continue;
        }

        // ====================================================
        // Header incompleto
        // ====================================================

        if (rx_length < 9U)
        {
            break;
        }

        // ====================================================
        // Flags
        // ====================================================

        const uint8_t flags =
            rx_buffer[8] &
            QUADMD_TELEMETRY_FULL;

        const size_t frame_size =
            quadmd_telemetry_frame_size(
                flags
            );

        // ====================================================
        // Frame incompleto
        // ====================================================

        if (
            rx_length <
            frame_size
        )
        {
            break;
        }

        // ====================================================
        // Trabalha em uma cópia.
        //
        // Isso preserva BATTERY/DIAGNOSTICS quando chegar FAST.
        // ====================================================

        quadmd_telemetry_t parsed;

        portENTER_CRITICAL(
            &telemetry_mux
        );

        parsed =
            telemetry;

        portEXIT_CRITICAL(
            &telemetry_mux
        );

        // ====================================================

        const bool ok =
            quadmd_parse_telemetry(
                rx_buffer,

                frame_size,

                ROBOT_ID,

                &parsed
            );

        // ====================================================

        if (ok)
        {
            const int64_t esp_rx_time_us =
                esp_timer_get_time();

            portENTER_CRITICAL(
                &telemetry_mux
            );

            telemetry =
                parsed;

            telemetry_packets++;

            portEXIT_CRITICAL(
                &telemetry_mux
            );

            /*
             * Copia snapshot para Web.
             */
            web_server_update_telemetry(
                &parsed,
                esp_rx_time_us
            );
        }
        else
        {
            portENTER_CRITICAL(
                &telemetry_mux
            );

            telemetry_parser_errors++;

            portEXIT_CRITICAL(
                &telemetry_mux
            );
        }

        // ====================================================

        remove_rx_bytes(
            frame_size
        );
    }
}

// ============================================================
// QUAD-MD TASK
// ============================================================

static void quadmd_task(
    void *arg)
{
    (void)arg;

    // ========================================================
    // D0
    // ========================================================

    const int64_t command_period =
        HZ_TO_US(
            QUADMD_COMMAND_HZ
        );

    // ========================================================
    // FULL
    // ========================================================

    const int64_t full_period =
        HZ_TO_US(
            QUADMD_TELEMETRY_FULL_HZ
        );

    // ========================================================

    int64_t now =
        esp_timer_get_time();

    int64_t last_command =
        now;

    int64_t last_full =
        now;

#if QUADMD_SPLIT_TELEMETRY

    // ========================================================
    // FAST
    // ========================================================

    const int64_t fast_period =
        HZ_TO_US(
            QUADMD_TELEMETRY_FAST_HZ
        );

    int64_t last_fast =
        now;

#endif

    // ========================================================

    while (1)
    {
        now =
            esp_timer_get_time();

        // ====================================================
        // RX continuamente
        // ====================================================

        process_rx();

        // ====================================================
        // D0
        // ====================================================

        if (
            now - last_command >=
            command_period
        )
        {
            last_command +=
                command_period;

            send_velocity();
        }

#if QUADMD_SPLIT_TELEMETRY

        // ====================================================
        // Um único E0 por ciclo FAST.
        //
        // Se estiver na hora do FULL,
        // o FULL substitui o FAST.
        // ====================================================

        if (
            now - last_fast >=
            fast_period
        )
        {
            last_fast +=
                fast_period;

            uint8_t flags =
                QUADMD_TELEMETRY_FAST;

            if (
                now - last_full >=
                full_period
            )
            {
                last_full +=
                    full_period;

                flags =
                    QUADMD_TELEMETRY_FULL;
            }

            request_telemetry(
                flags
            );
        }

#else

        // ====================================================
        // Perfil lento:
        //
        // apenas FULL.
        // ====================================================

        if (
            now - last_full >=
            full_period
        )
        {
            last_full +=
                full_period;

            request_telemetry(
                QUADMD_TELEMETRY_FULL
            );
        }

#endif

        // ====================================================
        // CPU
        // ====================================================

        vTaskDelay(
            pdMS_TO_TICKS(1)
        );
    }
}

// ============================================================
// MONITOR TASK
// ============================================================

static void monitor_task(
    void *arg)
{
    (void)arg;

    uint32_t previous_packets = 0;
    uint32_t previous_web_broadcasts = 0;

    while (1)
    {
        vTaskDelay(
            pdMS_TO_TICKS(1000)
        );

        // ====================================================
        // Snapshot
        // ====================================================

        quadmd_telemetry_t t;

        uint32_t current_packets;
        uint32_t parser_errors;
        uint32_t rx_overflows;

        portENTER_CRITICAL(
            &telemetry_mux
        );

        t =
            telemetry;

        current_packets =
            telemetry_packets;

        parser_errors =
            telemetry_parser_errors;

        rx_overflows =
            telemetry_rx_overflows;

        portEXIT_CRITICAL(
            &telemetry_mux
        );

        // ====================================================

        const uint32_t frequency =
            current_packets -
            previous_packets;

        previous_packets =
            current_packets;

        web_server_stats_t web_stats;

        web_server_get_stats(
            &web_stats
        );

        const uint32_t web_frequency =
            web_stats.telemetry_broadcasts -
            previous_web_broadcasts;

        previous_web_broadcasts =
            web_stats.telemetry_broadcasts;

        const uint32_t uart_errors =
            quadmd_uart_get_error_count() +
            rx_overflows;

        // ====================================================

        ESP_LOGI(
            TAG,

            "Telemetry RX: %lu Hz | "
            "Web TX: %lu Hz | "
            "Parser errors: %lu | "
            "Dropped: %lu | "
            "UART errors: %lu | "
            "Battery: %.3f V | "
            "RPM: %.1f %.1f %.1f %.1f | "
            "Comm: %u | "
            "Watchdog: %u | "
            "Fault: 0x%02X",

            (unsigned long)
                frequency,

            (unsigned long)
                web_frequency,

            (unsigned long)
                parser_errors,

            (unsigned long)
                web_stats.telemetry_dropped,

            (unsigned long)
                uart_errors,

            t.battery_voltage,

            t.rpm[0],
            t.rpm[1],
            t.rpm[2],
            t.rpm[3],

            t.communication_ok,

            t.watchdog_ok,

            t.fault_status
        );
    }
}

// ============================================================
// APP MAIN
// ============================================================

void app_main(void)
{
    ESP_LOGI(
        TAG,
        "================================"
    );

    ESP_LOGI(
        TAG,
        "TauraBots ESP32 Host"
    );

    ESP_LOGI(
        TAG,
        "================================"
    );

    // ========================================================
    // Perfil
    // ========================================================

    ESP_LOGI(
        TAG,
        "Robot ID: %c",
        ROBOT_ID
    );

    ESP_LOGI(
        TAG,
        "UART: %d baud",
        QUADMD_UART_BAUD
    );

    ESP_LOGI(
        TAG,
        "D0 command: %d Hz",
        QUADMD_COMMAND_HZ
    );

#if QUADMD_SPLIT_TELEMETRY

    ESP_LOGI(
        TAG,
        "E1 FAST: %d Hz",
        QUADMD_TELEMETRY_FAST_HZ
    );

#endif

    ESP_LOGI(
        TAG,
        "E1 FULL: %d Hz",
        QUADMD_TELEMETRY_FULL_HZ
    );

    ESP_LOGI(
        TAG,
        "Web telemetry: %d Hz",
        WEB_TELEMETRY_HZ
    );

    // ========================================================
    // CRC TEST
    // ========================================================

    const uint8_t test[] =
        "123456789";

    const uint16_t crc =
        quadmd_crc16(
            test,
            9U
        );

    ESP_LOGI(
        TAG,
        "CRC test: 0x%04X",
        crc
    );

    if (crc != 0x29B1U)
    {
        ESP_LOGE(
            TAG,
            "CRC TEST FAILED"
        );

        return;
    }

    // ========================================================
    // UART
    // ========================================================

    ESP_ERROR_CHECK(
        quadmd_uart_init()
    );

    ESP_LOGI(
        TAG,
        "Quad-MD UART ready"
    );

    // ========================================================
    // WiFi
    // ========================================================

    ESP_ERROR_CHECK(
        wifi_sta_init()
    );

    ESP_ERROR_CHECK(
        taura_mdns_start()
    );

    // ========================================================
    // Web
    // ========================================================

    ESP_ERROR_CHECK(
        web_server_start()
    );

    // ========================================================
    // Tasks
    // ========================================================

    BaseType_t ok;

    ok =
        xTaskCreatePinnedToCore(
            quadmd_task,

            "quadmd",

            4096,

            NULL,

            10,

            NULL,

            1
        );

    if (ok != pdPASS)
    {
        ESP_LOGE(
            TAG,
            "Falha criando quadmd_task"
        );

        return;
    }

    // ========================================================

    ok =
        xTaskCreatePinnedToCore(
            monitor_task,

            "monitor",

            4096,

            NULL,

            5,

            NULL,

            0
        );

    if (ok != pdPASS)
    {
        ESP_LOGE(
            TAG,
            "Falha criando monitor_task"
        );

        return;
    }

    ESP_LOGI(
        TAG,
        "Sistema iniciado"
    );
}
