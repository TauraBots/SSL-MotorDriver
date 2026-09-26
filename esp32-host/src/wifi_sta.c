#include "wifi_sta.h"
#include "mdns_service.h"

#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"

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
        ESP_LOGW(TAG, "STA desconectada: reason=%u, RSSI=%d dBm",
                 (unsigned int)disconnected->reason, (int)disconnected->rssi);
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
