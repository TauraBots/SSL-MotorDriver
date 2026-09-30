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
