#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

// ============================================================

typedef struct
{
	bool connected;
	int rssi;
	uint8_t channel;
} wifi_status_t;

const char *wifi_rssi_quality(int rssi);

bool wifi_sta_get_status(wifi_status_t *status);

esp_err_t wifi_sta_init(void);