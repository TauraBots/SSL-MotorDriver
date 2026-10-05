#pragma once

#include "comm_mode.h"
#include "main.h"

HAL_StatusTypeDef SerialService_Init(UART_HandleTypeDef *uart,
                                     DMA_HandleTypeDef *rx_dma);
void SerialService_Task(void);
void SerialService_OnTxComplete(UART_HandleTypeDef *uart);
void SerialService_OnError(UART_HandleTypeDef *uart);
void SerialService_SetCommMode(CommMode mode);
CommMode SerialService_GetCommMode(void);

extern volatile uint32_t match_telemetry_requests;
extern volatile uint32_t match_telemetry_responses;
extern volatile uint32_t match_discovery_requests;
extern volatile uint32_t match_discovery_responses;
extern volatile uint32_t match_uplink_dropped_busy;
extern volatile uint32_t match_config_discover_requests;
extern volatile uint32_t match_config_discover_responses;
extern volatile uint32_t match_config_set_id_requests;
extern volatile uint32_t match_config_set_id_responses;
extern volatile uint32_t match_config_motion_requests;
extern volatile uint32_t match_config_motion_responses;
