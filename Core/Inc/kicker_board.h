#ifndef KICKER_BOARD_H
#define KICKER_BOARD_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f1xx_hal.h"

#include <stdint.h>

#define KICKER_BOARD_I2C_ADDRESS_7BIT 0x42U
#define KICKER_BOARD_I2C_ADDRESS (KICKER_BOARD_I2C_ADDRESS_7BIT << 1U)
#define KICKER_BOARD_STATUS_SIZE 14U

typedef struct
{
  uint8_t valid;
  uint8_t flags;
  uint8_t sequence;
  uint16_t voltage_decivolt;
  uint16_t setpoint_decivolt;
  uint16_t auto_target_decivolt;
  uint16_t adc_raw;
  uint8_t result;
} KickerBoardStatus;

void KickerBoard_Init(I2C_HandleTypeDef *hi2c);
void KickerBoard_Task(void);

uint8_t KickerBoard_RequestAutoKick(uint8_t percent);
uint8_t KickerBoard_StartCharge(void);
uint8_t KickerBoard_StopCharge(void);
uint8_t KickerBoard_SetVoltage(uint16_t decivolt);
uint8_t KickerBoard_RequestImmediateKick(void);
uint8_t KickerBoard_ReadStatus(KickerBoardStatus *status);
uint8_t KickerBoard_Crc8(const uint8_t *data, uint16_t len);

/* Discards only a dangerous command that has not reached the I2C bus yet. */
void KickerBoard_CancelPendingKick(void);

extern volatile uint8_t kicker_dbg_online;
extern volatile uint8_t kicker_dbg_status_valid;
extern volatile uint16_t kicker_dbg_voltage_decivolt;
extern volatile uint16_t kicker_dbg_setpoint_decivolt;
extern volatile uint16_t kicker_dbg_auto_target_decivolt;
extern volatile uint8_t kicker_dbg_flags;
extern volatile uint8_t kicker_dbg_last_result;
extern volatile uint8_t kicker_dbg_status_sequence;
extern volatile uint32_t kicker_dbg_commands_sent;
extern volatile uint32_t kicker_dbg_kicks_requested;
extern volatile uint32_t kicker_dbg_kicks_sent;
extern volatile uint32_t kicker_dbg_i2c_errors;
extern volatile uint32_t kicker_dbg_crc_errors;
extern volatile uint32_t kicker_dbg_invalid_status;

#ifdef __cplusplus
}
#endif

#endif /* KICKER_BOARD_H */
