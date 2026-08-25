/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "app_c_api.h"
#include "stm32f1xx_hal_flash_ex.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define UART_RX_DMA_BUF_SIZE 256U
#define UART_TX_BUF_SIZE 192U
#define UART_TX_QUEUE_DEPTH 8U
#define TELEMETRY_SOF 0xAA55U
#define COMMAND_PACKET_LEN 19U
#define CONFIG_DISCOVER_PACKET_LEN 9U
#define CONFIG_SET_ID_PACKET_LEN 22U
#define CONFIG_RESPONSE_PACKET_LEN 23U
#define RX_FRAME_LEN_MAX CONFIG_SET_ID_PACKET_LEN
#define TELEMETRY_REQUEST_PACKET_LEN 10U
#define TELEMETRY_RESPONSE_PACKET_LEN 42U
#define RX_SOF0 0x55U
#define RX_SOF1 0xAAU
#define RX_TYPE_CONFIG_DISCOVER 0xF0U
#define RX_TYPE_CONFIG_SET_ID 0xF1U
#define TX_TYPE_CONFIG_DISCOVER_RESPONSE 0xF2U
#define TX_TYPE_CONFIG_SET_ID_RESPONSE 0xF3U
#define RX_TYPE_TELEMETRY_REQUEST 0xE0U
#define TX_TYPE_TELEMETRY_RESPONSE 0xE1U
#define TELEMETRY_PROTOCOL_VERSION 1U
#define TELEMETRY_MIN_INTERVAL_MS 100U
#define TELEMETRY_TURNAROUND_MS 3U
#define ROBOT_ID_BROADCAST ((uint8_t)'*')
#define ROBOT_CONFIG_ADDRESS 0x0803F800U
#define ROBOT_CONFIG_MAGIC 0x54425549U
#define ROBOT_CONFIG_VERSION 1U
#define ROBOT_CONFIG_KEY 0x46434449U
#define ROBOT_UID_LEN 12U

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c2;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim8;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;

/* USER CODE BEGIN PV */
static uint8_t uart_rx_dma_buf[UART_RX_DMA_BUF_SIZE];
static uint8_t uart_tx_buf[UART_TX_BUF_SIZE];
static uint8_t uart_tx_queue[UART_TX_QUEUE_DEPTH][UART_TX_BUF_SIZE];
static uint16_t uart_tx_queue_len[UART_TX_QUEUE_DEPTH];
static uint16_t uart_rx_last_pos = 0U;
static uint8_t uart_tx_q_head = 0U;
static uint8_t uart_tx_q_tail = 0U;
static uint8_t uart_tx_q_count = 0U;
static volatile uint8_t uart_tx_busy = 0U;
static volatile uint8_t uart_recovery_pending = 0U;
static volatile uint32_t uart_error_count = 0U;
static uint32_t last_telemetry_tick = 0U;
static uint32_t telemetry_due_tick = 0U;
static uint16_t telemetry_request_sequence = 0U;
static uint8_t telemetry_request_flags = 0U;
static uint8_t telemetry_response_pending = 0U;
static uint8_t telemetry_has_sent = 0U;
static uint32_t config_response_due_tick = 0U;
static uint8_t config_response_pending = 0U;
static uint8_t config_response_type = 0U;
static uint8_t config_response_status = 0U;

static volatile float vx = 0.0f;
static volatile float vy = 0.0f;
static volatile float w = 0.0f;
volatile float serial_dbg_rx_vx = 0.0f;
volatile float serial_dbg_rx_vy = 0.0f;
volatile float serial_dbg_rx_omega = 0.0f;
volatile uint32_t serial_dbg_rx_bytes = 0U;
volatile uint32_t serial_dbg_rx_frames = 0U;
volatile uint32_t serial_dbg_rx_bad_type = 0U;
volatile uint32_t serial_dbg_rx_bad_crc = 0U;
volatile uint16_t serial_dbg_crc_rx = 0U;
volatile uint16_t serial_dbg_crc_calc = 0U;

volatile float dbg_wheel_m1_rpm = 0.0f;
volatile float dbg_wheel_m2_rpm = 0.0f;
volatile float dbg_wheel_m3_rpm = 0.0f;
volatile float dbg_wheel_m4_rpm = 0.0f;

volatile float serial_dbg_applied_vx = 0.0f;
volatile float serial_dbg_applied_vy = 0.0f;
volatile float serial_dbg_applied_omega = 0.0f;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C2_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM8_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM5_Init(void);
static void MX_TIM6_Init(void);
/* USER CODE BEGIN PFP */
static void Serial_ProcessRx(void);
static void Serial_ProcessByte(uint8_t b);
static void Serial_ProcessCommandPacket(const uint8_t *buf);
static void Serial_TelemetryTask(void);
static void Serial_ConfigResponseTask(void);
static uint8_t Serial_QueueTx(const uint8_t *data, uint16_t len, uint8_t high_prio);
static void Serial_TxKick(void);
static void Serial_UartRecoveryTask(void);
static uint16_t Crc16CcittFalse(const uint8_t *data, uint16_t len);
static uint16_t U16LE(const uint8_t *p);
static uint32_t U32LE(const uint8_t *p);
static void WriteU32LE(uint8_t *p, uint32_t value);
static int16_t I16LE(const uint8_t *p);
static void RobotConfig_Init(void);
static uint8_t RobotConfig_SetId(uint8_t robot_id);
static void RobotUidRead(uint8_t uid[ROBOT_UID_LEN]);
static void Serial_ProcessDiscoverPacket(const uint8_t *buf);
static void Serial_ProcessSetIdPacket(const uint8_t *buf);
static void Serial_ProcessTelemetryRequest(const uint8_t *buf);
static uint32_t Crc32Ieee(const uint8_t *data, uint16_t len);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
typedef struct __attribute__((packed))
{
  uint16_t header;
  uint8_t robot_id;
  uint32_t sequence;
  int16_t motor1;
  int16_t motor2;
  int16_t motor3;
  int16_t motor4;
  uint8_t kick_power;
  uint8_t brake;
  uint16_t crc;
} CommandPacket;

typedef char CommandPacketSizeMustBe19Bytes[(sizeof(CommandPacket) == COMMAND_PACKET_LEN) ? 1 : -1];

typedef struct __attribute__((packed))
{
  uint32_t magic;
  uint16_t version;
  uint16_t size;
  uint8_t robot_id;
  uint8_t configured;
  uint8_t reserved[2];
  uint32_t generation;
  uint32_t crc32;
} RobotConfig;

typedef char RobotConfigSizeMustBe20Bytes[(sizeof(RobotConfig) == 20U) ? 1 : -1];

static const RobotConfig kDefaultRobotConfig = {
    ROBOT_CONFIG_MAGIC,
    ROBOT_CONFIG_VERSION,
    sizeof(RobotConfig),
    0U,
    0U,
    {0U, 0U},
    0U,
    0U};

volatile uint8_t live_robot_id = 0U;
volatile uint8_t live_robot_configured = 0U;
volatile uint32_t live_robot_uid_word0 = 0U;
volatile uint32_t live_robot_uid_word1 = 0U;
volatile uint32_t live_robot_uid_word2 = 0U;

typedef struct __attribute__((packed))
{
  uint16_t sof;
  uint8_t type;
  uint8_t version;
  uint8_t robot_id;
  uint8_t flags;
  uint8_t status;
  uint16_t request_sequence;
  uint32_t time_ms;
  uint32_t command_sequence;
  int16_t rpm1_x10;
  int16_t rpm2_x10;
  int16_t rpm3_x10;
  int16_t rpm4_x10;
  int16_t cmd1;
  int16_t cmd2;
  int16_t cmd3;
  int16_t cmd4;
  uint16_t battery_mv;
  uint16_t battery_adc;
  uint8_t brake;
  uint8_t communication_ok;
  uint8_t kick_power;
  uint16_t crc;
} TelemetryResponseFrame;

typedef char TelemetryResponseSizeMustBe42Bytes[(sizeof(TelemetryResponseFrame) == TELEMETRY_RESPONSE_PACKET_LEN) ? 1 : -1];

static uint16_t Crc16CcittFalse(const uint8_t *data, uint16_t len)
{
  uint16_t crc = 0xFFFFU;
  for (uint16_t i = 0U; i < len; i++)
  {
    crc ^= (uint16_t)data[i] << 8;
    for (uint8_t b = 0U; b < 8U; b++)
    {
      if ((crc & 0x8000U) != 0U)
      {
        crc = (uint16_t)((crc << 1) ^ 0x1021U);
      }
      else
      {
        crc <<= 1;
      }
    }
  }
  return crc;
}

static uint8_t RobotIdIsValid(uint8_t robot_id)
{
  return ((robot_id >= (uint8_t)'A') && (robot_id <= (uint8_t)'Z')) ? 1U : 0U;
}

static uint32_t Crc32Ieee(const uint8_t *data, uint16_t len)
{
  uint32_t crc = 0xFFFFFFFFU;
  for (uint16_t i = 0U; i < len; i++)
  {
    crc ^= data[i];
    for (uint8_t bit = 0U; bit < 8U; bit++)
    {
      crc = ((crc & 1U) != 0U) ? ((crc >> 1) ^ 0xEDB88320U) : (crc >> 1);
    }
  }
  return crc ^ 0xFFFFFFFFU;
}

static void RobotUidRead(uint8_t uid[ROBOT_UID_LEN])
{
  memcpy(uid, (const void *)UID_BASE, ROBOT_UID_LEN);
}

static void RobotConfig_Init(void)
{
  uint8_t uid[ROBOT_UID_LEN];
  RobotUidRead(uid);
  live_robot_uid_word0 = U32LE(&uid[0]);
  live_robot_uid_word1 = U32LE(&uid[4]);
  live_robot_uid_word2 = U32LE(&uid[8]);

  RobotConfig config = kDefaultRobotConfig;
  memcpy(&config, (const void *)ROBOT_CONFIG_ADDRESS, sizeof(config));
  const uint32_t crc = Crc32Ieee((const uint8_t *)&config, (uint16_t)(sizeof(config) - sizeof(config.crc32)));
  if ((config.magic == ROBOT_CONFIG_MAGIC) &&
      (config.version == ROBOT_CONFIG_VERSION) &&
      (config.size == sizeof(RobotConfig)) &&
      (config.configured == 1U) &&
      (RobotIdIsValid(config.robot_id) != 0U) &&
      (config.crc32 == crc))
  {
    live_robot_id = config.robot_id;
    live_robot_configured = 1U;
  }
}

static uint8_t RobotConfig_SetId(uint8_t robot_id)
{
  if (RobotIdIsValid(robot_id) == 0U)
  {
    return 0U;
  }

  RobotConfig config = kDefaultRobotConfig;
  config.robot_id = robot_id;
  config.configured = 1U;

  RobotConfig old_config;
  memcpy(&old_config, (const void *)ROBOT_CONFIG_ADDRESS, sizeof(old_config));
  const uint32_t old_crc = Crc32Ieee((const uint8_t *)&old_config, (uint16_t)(sizeof(old_config) - sizeof(old_config.crc32)));
  if ((old_config.magic == ROBOT_CONFIG_MAGIC) &&
      (old_config.version == ROBOT_CONFIG_VERSION) &&
      (old_config.size == sizeof(RobotConfig)) &&
      (old_config.crc32 == old_crc))
  {
    config.generation = old_config.generation + 1U;
  }
  else
  {
    config.generation = 1U;
  }
  config.crc32 = Crc32Ieee((const uint8_t *)&config, (uint16_t)(sizeof(config) - sizeof(config.crc32)));

  FLASH_EraseInitTypeDef erase = {0};
  uint32_t page_error = 0U;
  erase.TypeErase = FLASH_TYPEERASE_PAGES;
  erase.PageAddress = ROBOT_CONFIG_ADDRESS;
  erase.NbPages = 1U;

  if (HAL_FLASH_Unlock() != HAL_OK)
  {
    return 0U;
  }
  if (HAL_FLASHEx_Erase(&erase, &page_error) != HAL_OK)
  {
    (void)HAL_FLASH_Lock();
    return 0U;
  }

  for (uint32_t i = 0U; i < (sizeof(config) / sizeof(uint16_t)); i++)
  {
    const uint16_t halfword = U16LE(&((const uint8_t *)&config)[i * sizeof(uint16_t)]);
    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD,
                          ROBOT_CONFIG_ADDRESS + (i * sizeof(uint16_t)),
                          halfword) != HAL_OK)
    {
      (void)HAL_FLASH_Lock();
      return 0U;
    }
  }
  (void)HAL_FLASH_Lock();

  RobotConfig verify;
  memcpy(&verify, (const void *)ROBOT_CONFIG_ADDRESS, sizeof(verify));
  if (memcmp(&verify, &config, sizeof(config)) != 0)
  {
    return 0U;
  }

  live_robot_id = robot_id;
  live_robot_configured = 1U;
  return 1U;
}

static uint16_t U16LE(const uint8_t *p)
{
  return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}

static uint32_t U32LE(const uint8_t *p)
{
  return (uint32_t)p[0] |
         ((uint32_t)p[1] << 8) |
         ((uint32_t)p[2] << 16) |
         ((uint32_t)p[3] << 24);
}

static void WriteU32LE(uint8_t *p, uint32_t value)
{
  p[0] = (uint8_t)value;
  p[1] = (uint8_t)(value >> 8);
  p[2] = (uint8_t)(value >> 16);
  p[3] = (uint8_t)(value >> 24);
}

static int16_t I16LE(const uint8_t *p)
{
  return (int16_t)U16LE(p);
}

static void Serial_ProcessCommandPacket(const uint8_t *buf)
{
  const uint16_t crc_offset = COMMAND_PACKET_LEN - 2U;
  const uint16_t crc_rx = U16LE(&buf[crc_offset]);
  const uint16_t crc_ok = Crc16CcittFalse(buf, crc_offset);
  serial_dbg_crc_rx = crc_rx;
  serial_dbg_crc_calc = crc_ok;
  if (crc_rx != crc_ok)
  {
    serial_dbg_rx_bad_crc++;
    return;
  }

  serial_dbg_rx_frames++;
  const uint8_t robot_id = buf[2];
  if ((live_robot_configured == 0U) ||
      ((robot_id != live_robot_id) && (robot_id != ROBOT_ID_BROADCAST)))
  {
    return;
  }

  const uint32_t sequence = U32LE(&buf[3]);
  const int16_t motor1 = I16LE(&buf[7]);
  const int16_t motor2 = I16LE(&buf[9]);
  const int16_t motor3 = I16LE(&buf[11]);
  const int16_t motor4 = I16LE(&buf[13]);
  const uint8_t kick_power = buf[15];
  const uint8_t brake_mode = buf[16];

  dbg_wheel_m1_rpm = (float)motor1;
  dbg_wheel_m2_rpm = (float)motor2;
  dbg_wheel_m3_rpm = (float)motor3;
  dbg_wheel_m4_rpm = (float)motor4;
  AppC_SetCommands(
      sequence,
      (float)motor1, (float)motor2, (float)motor3, (float)motor4,
      kick_power, brake_mode);
}

static uint32_t RobotConfigGeneration(void)
{
  if (live_robot_configured == 0U)
  {
    return 0U;
  }
  RobotConfig config;
  memcpy(&config, (const void *)ROBOT_CONFIG_ADDRESS, sizeof(config));
  return config.generation;
}

static uint8_t Serial_SendConfigResponse(uint8_t type, uint8_t status)
{
  uint8_t response[CONFIG_RESPONSE_PACKET_LEN] = {0};
  uint8_t uid[ROBOT_UID_LEN];
  RobotUidRead(uid);
  response[0] = RX_SOF0;
  response[1] = RX_SOF1;
  response[2] = type;
  memcpy(&response[3], uid, ROBOT_UID_LEN);
  response[15] = status;
  response[16] = live_robot_id;
  WriteU32LE(&response[17], RobotConfigGeneration());
  const uint16_t crc = Crc16CcittFalse(response, CONFIG_RESPONSE_PACKET_LEN - 2U);
  response[21] = (uint8_t)crc;
  response[22] = (uint8_t)(crc >> 8);
  if ((uart_tx_busy != 0U) || (uart_tx_q_count != 0U))
  {
    return 0U;
  }

  return (HAL_UART_Transmit(&huart2, response, sizeof(response), 40U) == HAL_OK) ? 1U : 0U;
}

static void Serial_ProcessDiscoverPacket(const uint8_t *buf)
{
  const uint16_t crc_rx = U16LE(&buf[CONFIG_DISCOVER_PACKET_LEN - 2U]);
  const uint16_t crc_ok = Crc16CcittFalse(buf, CONFIG_DISCOVER_PACKET_LEN - 2U);
  if (crc_rx != crc_ok)
  {
    return;
  }

  uint8_t uid[ROBOT_UID_LEN];
  RobotUidRead(uid);
  const uint32_t nonce = U32LE(&buf[3]);
  uint32_t slot_hash = nonce ^ 0x9E3779B9U;
  for (uint8_t i = 0U; i < ROBOT_UID_LEN; i++)
  {
    slot_hash ^= uid[i];
    slot_hash *= 0x85EBCA6BU;
    slot_hash ^= slot_hash >> 13;
  }
  const uint32_t response_slot = slot_hash & 0x1FU;
  config_response_type = TX_TYPE_CONFIG_DISCOVER_RESPONSE;
  config_response_status = live_robot_configured;
  config_response_due_tick = HAL_GetTick() + (response_slot * 30U);
  config_response_pending = 1U;
}

static void Serial_ProcessSetIdPacket(const uint8_t *buf)
{
  const uint16_t crc_rx = U16LE(&buf[CONFIG_SET_ID_PACKET_LEN - 2U]);
  const uint16_t crc_ok = Crc16CcittFalse(buf, CONFIG_SET_ID_PACKET_LEN - 2U);
  uint8_t uid[ROBOT_UID_LEN];
  RobotUidRead(uid);
  if ((crc_rx != crc_ok) || (memcmp(&buf[3], uid, ROBOT_UID_LEN) != 0) ||
      (U32LE(&buf[16]) != ROBOT_CONFIG_KEY))
  {
    return;
  }

  uint8_t saved = 0U;
  AppC_ForceSafeState();
  saved = RobotConfig_SetId(buf[15]);
  config_response_type = TX_TYPE_CONFIG_SET_ID_RESPONSE;
  config_response_status = saved;
  config_response_due_tick = HAL_GetTick();
  config_response_pending = 1U;
}

static void Serial_ConfigResponseTask(void)
{
  if ((config_response_pending == 0U) ||
      ((int32_t)(HAL_GetTick() - config_response_due_tick) < 0))
  {
    return;
  }

  const uint8_t type = config_response_type;
  const uint8_t status = config_response_status;
  if (Serial_SendConfigResponse(type, status) != 0U)
  {
    config_response_pending = 0U;
  }
}

static void Serial_ProcessTelemetryRequest(const uint8_t *buf)
{
  const uint16_t crc_rx = U16LE(&buf[TELEMETRY_REQUEST_PACKET_LEN - 2U]);
  const uint16_t crc_ok = Crc16CcittFalse(buf, TELEMETRY_REQUEST_PACKET_LEN - 2U);
  const uint32_t now = HAL_GetTick();
  if ((crc_rx != crc_ok) ||
      (buf[3] != TELEMETRY_PROTOCOL_VERSION) ||
      (live_robot_configured == 0U) ||
      (buf[4] != live_robot_id) ||
      (telemetry_response_pending != 0U) ||
      ((telemetry_has_sent != 0U) && ((now - last_telemetry_tick) < TELEMETRY_MIN_INTERVAL_MS)))
  {
    return;
  }

  telemetry_request_sequence = U16LE(&buf[5]);
  telemetry_request_flags = buf[7];
  telemetry_due_tick = now + TELEMETRY_TURNAROUND_MS;
  telemetry_response_pending = 1U;
}

static void Serial_ProcessRx(void)
{
  uint16_t pos = UART_RX_DMA_BUF_SIZE - __HAL_DMA_GET_COUNTER(huart2.hdmarx);

  if (pos == uart_rx_last_pos)
  {
    return;
  }

  if (pos > uart_rx_last_pos)
  {
    for (uint16_t i = uart_rx_last_pos; i < pos; i++)
    {
      Serial_ProcessByte(uart_rx_dma_buf[i]);
    }
  }
  else
  {
    for (uint16_t i = uart_rx_last_pos; i < UART_RX_DMA_BUF_SIZE; i++)
    {
      Serial_ProcessByte(uart_rx_dma_buf[i]);
    }
    for (uint16_t i = 0U; i < pos; i++)
    {
      Serial_ProcessByte(uart_rx_dma_buf[i]);
    }
  }

  uart_rx_last_pos = pos;
}

static void Serial_ProcessByte(uint8_t b)
{
  serial_dbg_rx_bytes++;
  static uint8_t frame[RX_FRAME_LEN_MAX];
  static uint8_t idx = 0U;
  static uint8_t expected_len = COMMAND_PACKET_LEN;
  static uint8_t state = 0U;

  switch (state)
  {
    case 0U:
      if (b == RX_SOF0)
      {
        frame[0] = b;
        state = 1U;
      }
      break;

    case 1U:
      if (b == RX_SOF1)
      {
        frame[1] = b;
        idx = 2U;
        expected_len = COMMAND_PACKET_LEN;
        state = 2U;
      }
      else if (b == RX_SOF0)
      {
        frame[0] = b;
      }
      else
      {
        state = 0U;
      }
      break;

    case 2U:
      frame[idx++] = b;
      if (idx == 3U)
      {
        if (frame[2] == RX_TYPE_CONFIG_DISCOVER)
        {
          expected_len = CONFIG_DISCOVER_PACKET_LEN;
        }
        else if (frame[2] == RX_TYPE_CONFIG_SET_ID)
        {
          expected_len = CONFIG_SET_ID_PACKET_LEN;
        }
        else if (frame[2] == RX_TYPE_TELEMETRY_REQUEST)
        {
          expected_len = TELEMETRY_REQUEST_PACKET_LEN;
        }
        else
        {
          expected_len = COMMAND_PACKET_LEN;
        }
      }
      if (idx >= expected_len)
      {
        if (frame[2] == RX_TYPE_CONFIG_DISCOVER)
        {
          Serial_ProcessDiscoverPacket(frame);
        }
        else if (frame[2] == RX_TYPE_CONFIG_SET_ID)
        {
          Serial_ProcessSetIdPacket(frame);
        }
        else if (frame[2] == RX_TYPE_TELEMETRY_REQUEST)
        {
          Serial_ProcessTelemetryRequest(frame);
        }
        else
        {
          Serial_ProcessCommandPacket(frame);
        }
        state = 0U;
        idx = 0U;
        expected_len = COMMAND_PACKET_LEN;
      }
      break;

    default:
      state = 0U;
      idx = 0U;
      expected_len = COMMAND_PACKET_LEN;
      break;
  }
}

static uint8_t Serial_QueueTx(const uint8_t *data, uint16_t len, uint8_t high_prio)
{
  if ((data == NULL) || (len == 0U) || (len >= UART_TX_BUF_SIZE))
  {
    return 0U;
  }

  __disable_irq();
  if (uart_tx_q_count >= UART_TX_QUEUE_DEPTH)
  {
    __enable_irq();
    return 0U;
  }

  uint8_t idx = uart_tx_q_tail;
  if ((high_prio != 0U) && (uart_tx_q_count > 0U))
  {
    idx = (uart_tx_q_head == 0U) ? (UART_TX_QUEUE_DEPTH - 1U) : (uart_tx_q_head - 1U);
    uart_tx_q_head = idx;
  }
  else
  {
    uart_tx_q_tail = (uart_tx_q_tail + 1U) % UART_TX_QUEUE_DEPTH;
  }

  memcpy(uart_tx_queue[idx], data, len);
  uart_tx_queue_len[idx] = len;
  uart_tx_q_count++;
  __enable_irq();

  Serial_TxKick();
  return 1U;
}

static void Serial_TxKick(void)
{
  if ((uart_tx_busy != 0U) || (uart_tx_q_count == 0U))
  {
    return;
  }

  __disable_irq();
  if ((uart_tx_busy != 0U) || (uart_tx_q_count == 0U))
  {
    __enable_irq();
    return;
  }

  const uint8_t idx = uart_tx_q_head;
  const uint16_t len = uart_tx_queue_len[idx];
  memcpy(uart_tx_buf, uart_tx_queue[idx], len);
  uart_tx_q_head = (uart_tx_q_head + 1U) % UART_TX_QUEUE_DEPTH;
  uart_tx_q_count--;
  uart_tx_busy = 1U;
  __enable_irq();

  if (HAL_UART_Transmit_DMA(&huart2, uart_tx_buf, len) != HAL_OK)
  {
    uart_tx_busy = 0U;
    uart_recovery_pending = 1U;
  }
}

static void Serial_UartRecoveryTask(void)
{
  if (uart_recovery_pending == 0U)
  {
    return;
  }

  __disable_irq();
  uart_recovery_pending = 0U;
  uart_tx_busy = 0U;
  uart_tx_q_head = 0U;
  uart_tx_q_tail = 0U;
  uart_tx_q_count = 0U;
  telemetry_response_pending = 0U;
  __enable_irq();

  (void)HAL_UART_Abort(&huart2);
  __HAL_UART_CLEAR_OREFLAG(&huart2);
  uart_rx_last_pos = 0U;
  if (HAL_UART_Receive_DMA(&huart2, uart_rx_dma_buf, UART_RX_DMA_BUF_SIZE) == HAL_OK)
  {
    __HAL_DMA_DISABLE_IT(&hdma_usart2_rx, DMA_IT_HT);
  }
  else
  {
    uart_recovery_pending = 1U;
  }
}

static void Serial_TelemetryTask(void)
{
  if (telemetry_response_pending == 0U)
  {
    return;
  }

  const uint32_t now = HAL_GetTick();
  if ((int32_t)(now - telemetry_due_tick) < 0)
  {
    return;
  }

  AppC_Telemetry telem;
  AppC_GetTelemetry(&telem);
  TelemetryResponseFrame frame;
  memset(&frame, 0, sizeof(frame));
  frame.sof = TELEMETRY_SOF;
  frame.type = TX_TYPE_TELEMETRY_RESPONSE;
  frame.version = TELEMETRY_PROTOCOL_VERSION;
  frame.robot_id = live_robot_id;
  frame.flags = telemetry_request_flags;
  frame.status = 1U;
  frame.request_sequence = telemetry_request_sequence;
  frame.time_ms = telem.time_ms;
  frame.command_sequence = telem.last_command_sequence;
  frame.rpm1_x10 = (int16_t)(telem.rpm_m1 * 10.0f);
  frame.rpm2_x10 = (int16_t)(telem.rpm_m2 * 10.0f);
  frame.rpm3_x10 = (int16_t)(telem.rpm_m3 * 10.0f);
  frame.rpm4_x10 = (int16_t)(telem.rpm_m4 * 10.0f);
  frame.cmd1 = (int16_t)telem.cmd_m1;
  frame.cmd2 = (int16_t)telem.cmd_m2;
  frame.cmd3 = (int16_t)telem.cmd_m3;
  frame.cmd4 = (int16_t)telem.cmd_m4;
  frame.battery_mv = (uint16_t)(telem.battery_voltage_v * 1000.0f);
  frame.battery_adc = (uint16_t)telem.battery_adc_raw;
  frame.brake = telem.stop_mode_brake;
  frame.communication_ok = telem.communication_ok;
  frame.kick_power = telem.kick_power;
  frame.crc = Crc16CcittFalse((const uint8_t *)&frame, (uint16_t)(sizeof(frame) - sizeof(frame.crc)));

  if (Serial_QueueTx((const uint8_t *)&frame, (uint16_t)sizeof(frame), 0U) != 0U)
  {
    last_telemetry_tick = now;
    telemetry_has_sent = 1U;
    telemetry_response_pending = 0U;
  }
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_I2C2_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_TIM8_Init();
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */
  RobotConfig_Init();
  AppC_Init(&hadc1, &htim1, &htim8, &htim5, &htim3, &htim2, &htim4, LED_GPIO_Port, LED_Pin);
  HAL_TIM_Base_Start_IT(&htim6);
  if (HAL_UART_Receive_DMA(&huart2, uart_rx_dma_buf, UART_RX_DMA_BUF_SIZE) != HAL_OK)
  {
    Error_Handler();
  }
  __HAL_DMA_DISABLE_IT(&hdma_usart2_rx, DMA_IT_HT);
  HAL_UART_Transmit(&huart2, (uint8_t *)"USART2 READY\r\n", strlen("USART2 READY\r\n"), 20U);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    AppC_Tick();
    Serial_UartRecoveryTask();
    Serial_ProcessRx();
    Serial_ConfigResponseTask();
    Serial_TelemetryTask();
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_14;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 2399;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_LOW;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim4, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 0;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 65535;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim5, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 71;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 999;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 0;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 2399;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim8, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */
  HAL_TIM_MspPostInit(&htim8);

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel6_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel6_IRQn);
  /* DMA1_Channel7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel7_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel7_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM6)
  {
    AppC_FastTick1kHz();
  }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART2)
  {
    uart_tx_busy = 0U;
    Serial_TxKick();
  }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART2)
  {
    uart_error_count++;
    uart_tx_busy = 0U;
    uart_recovery_pending = 1U;
  }
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
