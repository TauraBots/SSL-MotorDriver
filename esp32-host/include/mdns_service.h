#pragma once

#include <stddef.h>

#include "esp_err.h"

#define TAURA_HOSTNAME_SIZE sizeof("robot-a")

/* Shared DHCP/mDNS hostname, derived from TAURA_ROBOT_ID (without .local). */
esp_err_t taura_get_hostname(char *buffer, size_t size);

/* Call after wifi_sta_init() has obtained an IP address, before HTTP startup. */
esp_err_t taura_mdns_start(void);
