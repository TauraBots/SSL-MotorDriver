#include "mdns_service.h"

#include <ctype.h>
#include <stdio.h>

#include "esp_log.h"
#include "mdns.h"

#include "app_config.h"

static const char *TAG = "MDNS";

_Static_assert((TAURA_ROBOT_ID >= 'A' && TAURA_ROBOT_ID <= 'Z') ||
               (TAURA_ROBOT_ID >= 'a' && TAURA_ROBOT_ID <= 'z') ||
               (TAURA_ROBOT_ID >= '0' && TAURA_ROBOT_ID <= '9'),
               "TAURA_ROBOT_ID must be an ASCII letter or digit for robot-x.local");

esp_err_t taura_get_hostname(char *buffer, size_t size)
{
    if (buffer == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (size < TAURA_HOSTNAME_SIZE) {
        return ESP_ERR_INVALID_SIZE;
    }
    snprintf(buffer, size, "robot-%c", tolower((unsigned char)TAURA_ROBOT_ID));
    return ESP_OK;
}

esp_err_t taura_mdns_start(void)
{
    char hostname[TAURA_HOSTNAME_SIZE];
    esp_err_t err = taura_get_hostname(hostname, sizeof(hostname));
    if (err != ESP_OK) {
        return err;
    }
    err = mdns_init();
    if (err != ESP_OK) {
        return err;
    }
    err = mdns_hostname_set(hostname);
    if (err == ESP_OK) {
        err = mdns_instance_name_set(hostname);
    }
    if (err == ESP_OK) {
        err = mdns_service_add(NULL, "_http", "_tcp", 80, NULL, 0);
    }
    if (err != ESP_OK) {
        mdns_free();
        return err;
    }
    ESP_LOGI(TAG, "Hostname: %s", hostname);
    ESP_LOGI(TAG, "mDNS: http://%s.local", hostname);
    return ESP_OK;
}
