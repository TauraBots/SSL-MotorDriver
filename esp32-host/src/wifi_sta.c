#include "wifi_sta.h"
#include "mdns_service.h"
#include "app_config.h"

#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/task.h"

#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_wifi.h"

#include "nvs_flash.h"

#include "wifi_secrets.h"

// ============================================================

static const char *TAG =
    "WIFI";

// ============================================================

#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

#define WIFI_MAX_RETRY     10

// ============================================================

static EventGroupHandle_t
    wifi_event_group;

static int
    retry_count = 0;

static wifi_status_t
    wifi_status =
        {
            .rssi = -127
        };

static portMUX_TYPE
    wifi_status_mux =
        portMUX_INITIALIZER_UNLOCKED;

// Project diagnostic labels, not an official Espressif classification.
const char *wifi_rssi_quality(int rssi)
{
    if (rssi >= -50)
    {
        return "EXCELENTE";
    }

    if (rssi >= -59)
    {
        return "FORTE";
    }

    if (rssi >= -67)
    {
        return "BOM";
    }

    if (rssi >= -75)
    {
        return "FRACO";
    }

    return "CRITICO";
}

bool wifi_sta_get_status(wifi_status_t *status)
{
    if (status == NULL)
    {
        return false;
    }

    portENTER_CRITICAL(&wifi_status_mux);
    *status = wifi_status;
    portEXIT_CRITICAL(&wifi_status_mux);

    return status->connected;
}

static void wifi_status_set_connected(bool connected)
{
    portENTER_CRITICAL(&wifi_status_mux);
    wifi_status.connected = connected;
    portEXIT_CRITICAL(&wifi_status_mux);
}

static void wifi_status_update_ap(const wifi_ap_record_t *ap_info)
{
    portENTER_CRITICAL(&wifi_status_mux);
    wifi_status.connected = true;
    wifi_status.rssi = ap_info->rssi;
    wifi_status.channel = ap_info->primary;
    portEXIT_CRITICAL(&wifi_status_mux);
}

static void wifi_monitor_task(void *arg)
{
    (void)arg;

    bool ap_logged = false;

    while (1)
    {
        vTaskDelay(pdMS_TO_TICKS(WIFI_MONITOR_PERIOD_MS));

        wifi_status_t status;
        (void)wifi_sta_get_status(&status);

        if (!status.connected)
        {
            ap_logged = false;
            continue;
        }

        wifi_ap_record_t ap_info;
        if (esp_wifi_sta_get_ap_info(&ap_info) != ESP_OK)
        {
            continue;
        }

        wifi_status_update_ap(&ap_info);
        ESP_LOGI(TAG, "RSSI: %d dBm | Sinal: %s",
                 ap_info.rssi, wifi_rssi_quality(ap_info.rssi));

        if (!ap_logged)
        {
            ESP_LOGI(TAG, "SSID: %.*s | Channel: %u",
                     (int)sizeof(ap_info.ssid), ap_info.ssid,
                     (unsigned int)ap_info.primary);
            ap_logged = true;
        }
    }
}

// ============================================================
// EVENT HANDLER
// ============================================================

static void wifi_event_handler(
    void *arg,
    esp_event_base_t event_base,
    int32_t event_id,
    void *event_data)
{
    (void)arg;

    // ========================================================
    // Wi-Fi iniciou
    // ========================================================

    if (
        event_base == WIFI_EVENT &&
        event_id == WIFI_EVENT_STA_START
    )
    {
        ESP_LOGI(
            TAG,
            "Conectando em \"%s\"...",
            TAURA_WIFI_SSID
        );

        esp_wifi_connect();

        return;
    }

    // ========================================================
    // Desconectou
    // ========================================================

    if (
        event_base == WIFI_EVENT &&
        event_id ==
            WIFI_EVENT_STA_DISCONNECTED
    )
    {
        const wifi_event_sta_disconnected_t *disconnected = event_data;
        wifi_status_t previous;
        (void)wifi_sta_get_status(&previous);
        wifi_status_set_connected(false);
        ESP_LOGW(TAG, "DESCONECTADO");
        ESP_LOGW(TAG, "reason=%u", (unsigned int)disconnected->reason);
        ESP_LOGW(TAG, "ultimo RSSI=%d dBm | sinal anterior=%s",
                 previous.rssi, wifi_rssi_quality(previous.rssi));
        if (
            retry_count <
            WIFI_MAX_RETRY
        )
        {
            retry_count++;

            ESP_LOGW(
                TAG,
                "WiFi desconectado "
                "(tentativa %d/%d)",

                retry_count,
                WIFI_MAX_RETRY
            );

            esp_wifi_connect();
        }
        else
        {
            ESP_LOGE(
                TAG,
                "Limite de tentativas atingido"
            );

            xEventGroupSetBits(
                wifi_event_group,
                WIFI_FAIL_BIT
            );
        }

        return;
    }

    // ========================================================
    // Recebeu IP
    // ========================================================

    if (
        event_base == IP_EVENT &&
        event_id == IP_EVENT_STA_GOT_IP
    )
    {
        ip_event_got_ip_t *event =
            (ip_event_got_ip_t *)
            event_data;

        retry_count = 0;

        wifi_status_set_connected(true);

        ESP_LOGI(
            TAG,
            "WiFi conectado"
        );

        ESP_LOGI(
            TAG,
            "IP direto: http://" IPSTR,
            IP2STR(
                &event->ip_info.ip
            )
        );

        ESP_LOGI(
            TAG,
            "Gateway: " IPSTR,
            IP2STR(
                &event->ip_info.gw
            )
        );

        xEventGroupSetBits(
            wifi_event_group,
            WIFI_CONNECTED_BIT
        );
    }
}

// ============================================================

esp_err_t wifi_sta_init(void)
{
    // ========================================================
    // NVS
    // ========================================================

    esp_err_t ret =
        nvs_flash_init();

    if (
        ret ==
            ESP_ERR_NVS_NO_FREE_PAGES ||

        ret ==
            ESP_ERR_NVS_NEW_VERSION_FOUND
    )
    {
        ESP_ERROR_CHECK(
            nvs_flash_erase()
        );

        ret =
            nvs_flash_init();
    }

    ESP_ERROR_CHECK(ret);

    // ========================================================
    // TCP/IP
    // ========================================================

    ESP_ERROR_CHECK(
        esp_netif_init()
    );

    // ========================================================
    // Event loop
    // ========================================================

    ret =
        esp_event_loop_create_default();

    if (
        ret != ESP_OK &&
        ret != ESP_ERR_INVALID_STATE
    )
    {
        return ret;
    }

    // ========================================================
    // STA interface
    // ========================================================

    esp_netif_t *sta =
        esp_netif_create_default_wifi_sta();

    if (sta == NULL)
    {
        return ESP_FAIL;
    }

    char hostname[TAURA_HOSTNAME_SIZE];
    ESP_ERROR_CHECK(taura_get_hostname(hostname, sizeof(hostname)));

    ESP_ERROR_CHECK(
        esp_netif_set_hostname(
            sta,
            hostname
        )
    );

    // ========================================================
    // WiFi init
    // ========================================================

    wifi_init_config_t cfg =
        WIFI_INIT_CONFIG_DEFAULT();

    ESP_ERROR_CHECK(
        esp_wifi_init(
            &cfg
        )
    );

    wifi_event_group =
        xEventGroupCreate();

    if (wifi_event_group == NULL)
    {
        return ESP_ERR_NO_MEM;
    }

    // ========================================================
    // Eventos
    // ========================================================

    ESP_ERROR_CHECK(
        esp_event_handler_register(
            WIFI_EVENT,
            ESP_EVENT_ANY_ID,
            wifi_event_handler,
            NULL
        )
    );

    ESP_ERROR_CHECK(
        esp_event_handler_register(
            IP_EVENT,
            IP_EVENT_STA_GOT_IP,
            wifi_event_handler,
            NULL
        )
    );

    // ========================================================
    // Config
    // ========================================================

    wifi_config_t wifi_config =
        {0};

    strncpy(
        (char *)
            wifi_config.sta.ssid,

        TAURA_WIFI_SSID,

        sizeof(
            wifi_config.sta.ssid
        ) - 1U
    );

    strncpy(
        (char *)
            wifi_config.sta.password,

        TAURA_WIFI_PASSWORD,

        sizeof(
            wifi_config.sta.password
        ) - 1U
    );

    // ========================================================
    // STA
    // ========================================================

    ESP_ERROR_CHECK(
        esp_wifi_set_mode(
            WIFI_MODE_STA
        )
    );

    ESP_ERROR_CHECK(
        esp_wifi_set_config(
            WIFI_IF_STA,
            &wifi_config
        )
    );

    ESP_ERROR_CHECK(
        esp_wifi_start()
    );
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));

    ESP_LOGI(TAG, "Power save: DISABLED");
    ESP_LOGI(TAG, "TX power requested: %.1f dBm",
             WIFI_TX_POWER_DBM / 1.0f);
    ESP_ERROR_CHECK(
        esp_wifi_set_max_tx_power(
            WIFI_TX_POWER_QUARTER_DBM
        )
    );

    int8_t tx_power = 0;
    ESP_ERROR_CHECK(
        esp_wifi_get_max_tx_power(
            &tx_power
        )
    );
    ESP_LOGI(TAG, "TX power active: %.2f dBm",
             tx_power / 4.0f);

    // ========================================================
    // Aguarda resultado
    // ========================================================

    const EventBits_t bits =
        xEventGroupWaitBits(
            wifi_event_group,

            WIFI_CONNECTED_BIT |
            WIFI_FAIL_BIT,

            pdFALSE,
            pdFALSE,

            portMAX_DELAY
        );

    // ========================================================

    if (
        bits &
        WIFI_CONNECTED_BIT
    )
    {
        if (xTaskCreate(
                wifi_monitor_task,
                "wifi_monitor",
                3072,
                NULL,
                4,
                NULL) != pdPASS)
        {
            return ESP_ERR_NO_MEM;
        }

        ESP_LOGI(
            TAG,
            "Conectado em \"%s\"",
            TAURA_WIFI_SSID
        );

        return ESP_OK;
    }

    ESP_LOGE(
        TAG,
        "Falha ao conectar em \"%s\"",
        TAURA_WIFI_SSID
    );

    return ESP_FAIL;
}
