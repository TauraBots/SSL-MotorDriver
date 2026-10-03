#include "kicker_board.h"

#include <string.h>

#define KICKER_COMMAND_SELECT_STATUS 0x00U
#define KICKER_COMMAND_START_CHARGE 0x01U
#define KICKER_COMMAND_STOP_CHARGE 0x02U
#define KICKER_COMMAND_IMMEDIATE_KICK 0x03U
#define KICKER_COMMAND_SET_VOLTAGE 0x04U
#define KICKER_COMMAND_AUTO_KICK 0x05U

#define KICKER_STATUS_HEADER 0xA5U
#define KICKER_STATUS_VERSION 0x01U
#define KICKER_I2C_TX_TIMEOUT_MS 2U
#define KICKER_I2C_STATUS_TIMEOUT_MS 10U
#define KICKER_STATUS_PERIOD_MS 100U

typedef enum
{
  KICKER_STATE_IDLE = 0,
  KICKER_STATE_AUTO_KICK_PENDING,
  KICKER_STATE_IMMEDIATE_KICK_PENDING,
  KICKER_STATE_SAFE_STOP_PENDING,
  KICKER_STATE_WAITING_STATUS,
  KICKER_STATE_ERROR
} KickerBoardState;

volatile uint8_t kicker_dbg_online = 0U;
volatile uint8_t kicker_dbg_status_valid = 0U;
volatile uint16_t kicker_dbg_voltage_decivolt = 0U;
volatile uint16_t kicker_dbg_setpoint_decivolt = 0U;
volatile uint16_t kicker_dbg_auto_target_decivolt = 0U;
volatile uint8_t kicker_dbg_flags = 0U;
volatile uint8_t kicker_dbg_last_result = 0U;
volatile uint8_t kicker_dbg_status_sequence = 0U;
volatile uint32_t kicker_dbg_commands_sent = 0U;
volatile uint32_t kicker_dbg_kicks_requested = 0U;
volatile uint32_t kicker_dbg_kicks_sent = 0U;
volatile uint32_t kicker_dbg_i2c_errors = 0U;
volatile uint32_t kicker_dbg_crc_errors = 0U;
volatile uint32_t kicker_dbg_invalid_status = 0U;
volatile uint32_t kicker_dbg_task_calls = 0U;
volatile uint32_t kicker_dbg_status_reads = 0U;

static I2C_HandleTypeDef *kicker_i2c = NULL;
static KickerBoardState kicker_state = KICKER_STATE_IDLE;
static KickerBoardStatus kicker_status;
static uint8_t pending_auto_percent = 0U;
static uint32_t last_status_attempt_ms = 0U;

static uint16_t ReadLe16(const uint8_t *data)
{
  return (uint16_t)data[0] | ((uint16_t)data[1] << 8U);
}

static uint8_t SendCommand(uint8_t *command, uint16_t size)
{
  if (kicker_i2c == NULL)
  {
    return 0U;
  }

  if (HAL_I2C_Master_Transmit(kicker_i2c, KICKER_BOARD_I2C_ADDRESS,
                              command, size, KICKER_I2C_TX_TIMEOUT_MS) != HAL_OK)
  {
    kicker_dbg_online = 0U;
    kicker_dbg_i2c_errors++;
    return 0U;
  }

  kicker_dbg_commands_sent++;
  return 1U;
}

uint8_t KickerBoard_Crc8(const uint8_t *data, uint16_t len)
{
  uint8_t crc = 0U;
  uint16_t i;

  if (data == NULL)
  {
    return 0U;
  }

  for (i = 0U; i < len; i++)
  {
    uint8_t bit;
    crc ^= data[i];
    for (bit = 0U; bit < 8U; bit++)
    {
      crc = ((crc & 0x80U) != 0U) ?
          (uint8_t)((crc << 1U) ^ 0x07U) : (uint8_t)(crc << 1U);
    }
  }
  return crc;
}

void KickerBoard_Init(I2C_HandleTypeDef *hi2c)
{
  kicker_i2c = hi2c;
  kicker_state = KICKER_STATE_IDLE;
  pending_auto_percent = 0U;
  last_status_attempt_ms = HAL_GetTick();
  memset(&kicker_status, 0, sizeof(kicker_status));

  kicker_dbg_online = 0U;
  kicker_dbg_status_valid = 0U;
  kicker_dbg_voltage_decivolt = 0U;
  kicker_dbg_setpoint_decivolt = 0U;
  kicker_dbg_auto_target_decivolt = 0U;
  kicker_dbg_flags = 0U;
  kicker_dbg_last_result = 0U;
  kicker_dbg_status_sequence = 0U;
  kicker_dbg_commands_sent = 0U;
  kicker_dbg_kicks_requested = 0U;
  kicker_dbg_kicks_sent = 0U;
  kicker_dbg_i2c_errors = 0U;
  kicker_dbg_crc_errors = 0U;
  kicker_dbg_invalid_status = 0U;
  kicker_dbg_task_calls = 0U;
  kicker_dbg_status_reads = 0U;
}

uint8_t KickerBoard_RequestAutoKick(uint8_t percent)
{
  if ((percent < 1U) || (percent > 100U) ||
      (kicker_i2c == NULL) || (kicker_state != KICKER_STATE_IDLE))
  {
    return 0U;
  }

  pending_auto_percent = percent;
  kicker_state = KICKER_STATE_AUTO_KICK_PENDING;
  kicker_dbg_kicks_requested++;
  return 1U;
}

uint8_t KickerBoard_RequestImmediateKick(void)
{
  if ((kicker_i2c == NULL) || (kicker_state != KICKER_STATE_IDLE))
  {
    return 0U;
  }

  kicker_state = KICKER_STATE_IMMEDIATE_KICK_PENDING;
  kicker_dbg_kicks_requested++;
  return 1U;
}

uint8_t KickerBoard_RequestSafeStop(void)
{
  if (kicker_i2c == NULL)
  {
    return 0U;
  }

  /* A safe stop supersedes every pending or uncertain operation. */
  pending_auto_percent = 0U;
  kicker_state = KICKER_STATE_SAFE_STOP_PENDING;
  return 1U;
}

void KickerBoard_CancelPendingKick(void)
{
  if ((kicker_state == KICKER_STATE_AUTO_KICK_PENDING) ||
      (kicker_state == KICKER_STATE_IMMEDIATE_KICK_PENDING))
  {
    pending_auto_percent = 0U;
    kicker_state = KICKER_STATE_IDLE;
  }
}

uint8_t KickerBoard_StartCharge(void)
{
  uint8_t command = KICKER_COMMAND_START_CHARGE;
  return SendCommand(&command, 1U);
}

uint8_t KickerBoard_StopCharge(void)
{
  uint8_t command = KICKER_COMMAND_STOP_CHARGE;
  return SendCommand(&command, 1U);
}

uint8_t KickerBoard_SetVoltage(uint16_t decivolt)
{
  uint8_t command[3];
  if ((decivolt < 200U) || (decivolt > 2000U))
  {
    return 0U;
  }

  command[0] = KICKER_COMMAND_SET_VOLTAGE;
  command[1] = (uint8_t)(decivolt & 0xFFU);
  command[2] = (uint8_t)(decivolt >> 8U);
  return SendCommand(command, sizeof(command));
}

uint8_t KickerBoard_ReadStatus(KickerBoardStatus *status)
{
  uint8_t buffer[KICKER_BOARD_STATUS_SIZE];

  if ((status == NULL) || (kicker_i2c == NULL))
  {
    return 0U;
  }

  last_status_attempt_ms = HAL_GetTick();
  status->valid = 0U;
  kicker_dbg_status_valid = 0U;
  kicker_dbg_status_reads++;
  if (HAL_I2C_Master_Receive(kicker_i2c, KICKER_BOARD_I2C_ADDRESS,
                             buffer, sizeof(buffer),
                             KICKER_I2C_STATUS_TIMEOUT_MS) != HAL_OK)
  {
    kicker_dbg_online = 0U;
    kicker_dbg_i2c_errors++;
    return 0U;
  }

  if ((buffer[0] != KICKER_STATUS_HEADER) ||
      (buffer[1] != KICKER_STATUS_VERSION))
  {
    kicker_dbg_invalid_status++;
    return 0U;
  }

  if (KickerBoard_Crc8(buffer, KICKER_BOARD_STATUS_SIZE - 1U) != buffer[13])
  {
    kicker_dbg_crc_errors++;
    kicker_dbg_invalid_status++;
    return 0U;
  }

  status->flags = buffer[2];
  status->sequence = buffer[3];
  status->voltage_decivolt = ReadLe16(&buffer[4]);
  status->setpoint_decivolt = ReadLe16(&buffer[6]);
  status->auto_target_decivolt = ReadLe16(&buffer[8]);
  status->adc_raw = ReadLe16(&buffer[10]);
  status->result = buffer[12];
  status->valid = 1U;

  kicker_dbg_online = 1U;
  kicker_dbg_status_valid = 1U;
  kicker_dbg_voltage_decivolt = status->voltage_decivolt;
  kicker_dbg_setpoint_decivolt = status->setpoint_decivolt;
  kicker_dbg_auto_target_decivolt = status->auto_target_decivolt;
  kicker_dbg_flags = status->flags;
  kicker_dbg_last_result = status->result;
  kicker_dbg_status_sequence = status->sequence;
  return 1U;
}

void KickerBoard_Task(void)
{
  kicker_dbg_task_calls++;
  const uint32_t now = HAL_GetTick();

  if (kicker_i2c == NULL)
  {
    return;
  }

  if (kicker_state == KICKER_STATE_AUTO_KICK_PENDING)
  {
    uint8_t command[2] = {KICKER_COMMAND_AUTO_KICK, pending_auto_percent};

    /* Exactly one transmit attempt. Both success and failure move forward. */
    if (SendCommand(command, sizeof(command)) != 0U)
    {
      kicker_dbg_kicks_sent++;
    }
    pending_auto_percent = 0U;
    kicker_status.valid = 0U;
    kicker_dbg_status_valid = 0U;
    kicker_state = KICKER_STATE_WAITING_STATUS;
    return;
  }

  if (kicker_state == KICKER_STATE_IMMEDIATE_KICK_PENDING)
  {
    uint8_t command = KICKER_COMMAND_IMMEDIATE_KICK;

    /* Exactly one transmit attempt. Never retry an ambiguous kick. */
    if (SendCommand(&command, 1U) != 0U)
    {
      kicker_dbg_kicks_sent++;
    }
    kicker_status.valid = 0U;
    kicker_dbg_status_valid = 0U;
    kicker_state = KICKER_STATE_WAITING_STATUS;
    return;
  }

  if (kicker_state == KICKER_STATE_SAFE_STOP_PENDING)
  {
    uint8_t command = KICKER_COMMAND_STOP_CHARGE;

    /* One attempt per safe-stop event; no automatic transmit retry. */
    (void)SendCommand(&command, 1U);
    kicker_status.valid = 0U;
    kicker_dbg_status_valid = 0U;
    kicker_state = KICKER_STATE_WAITING_STATUS;
    return;
  }

  if (kicker_state == KICKER_STATE_WAITING_STATUS)
  {
    kicker_state = (KickerBoard_ReadStatus(&kicker_status) != 0U) ?
        KICKER_STATE_IDLE : KICKER_STATE_ERROR;
    return;
  }

  if (((kicker_state == KICKER_STATE_IDLE) ||
       (kicker_state == KICKER_STATE_ERROR)) &&
      ((now - last_status_attempt_ms) >= KICKER_STATUS_PERIOD_MS))
  {
    kicker_state = (KickerBoard_ReadStatus(&kicker_status) != 0U) ?
        KICKER_STATE_IDLE : KICKER_STATE_ERROR;
  }
}
