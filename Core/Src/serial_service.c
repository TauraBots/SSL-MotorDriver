#include "serial_service.h"

#include "app_c_api.h"
#include "crsf_protocol.h"
#include "match_control.h"
#include "robot_identity.h"
#include "serial_profile.h"
#include "serial_protocol.h"
#include <stdio.h>
#include <string.h>

#define UART_RX_DMA_BUF_SIZE 256U
#define UART_TX_BUF_SIZE 192U
#define UART_TX_QUEUE_DEPTH 8U
#define TELEMETRY_SOF 0xAA55U
#define COMMAND_PACKET_LEN SERIAL_COMMAND_PACKET_LEN
#define ROBOT_VELOCITY_PACKET_LEN SERIAL_ROBOT_VELOCITY_PACKET_LEN
#define CONFIG_DISCOVER_PACKET_LEN SERIAL_CONFIG_DISCOVER_PACKET_LEN
#define CONFIG_SET_ID_PACKET_LEN SERIAL_CONFIG_SET_ID_PACKET_LEN
#define CONFIG_RESPONSE_PACKET_LEN SERIAL_CONFIG_RESPONSE_PACKET_LEN
#define CONFIG_SET_MOTION_PACKET_LEN SERIAL_CONFIG_SET_MOTION_PACKET_LEN
#define RX_FRAME_LEN_MAX SERIAL_CONFIG_SET_MOTION_PACKET_LEN
#define TELEMETRY_REQUEST_PACKET_LEN SERIAL_TELEMETRY_REQUEST_PACKET_LEN
#define TELEMETRY_RESPONSE_PACKET_LEN_MAX SERIAL_TELEMETRY_RESPONSE_PACKET_LEN_MAX
#define DISCOVERY_REQUEST_PACKET_LEN SERIAL_DISCOVERY_REQUEST_PACKET_LEN
#define DISCOVERY_RESPONSE_PACKET_LEN SERIAL_DISCOVERY_RESPONSE_PACKET_LEN
#define RX_SOF0 SERIAL_RX_SOF0
#define RX_SOF1 SERIAL_RX_SOF1
#define RX_TYPE_CONFIG_DISCOVER SERIAL_TYPE_CONFIG_DISCOVER
#define RX_TYPE_CONFIG_SET_ID SERIAL_TYPE_CONFIG_SET_ID
#define TX_TYPE_CONFIG_DISCOVER_RESPONSE SERIAL_TYPE_CONFIG_DISCOVER_RESPONSE
#define TX_TYPE_CONFIG_SET_ID_RESPONSE SERIAL_TYPE_CONFIG_SET_ID_RESPONSE
#define RX_TYPE_CONFIG_SET_MOTION SERIAL_TYPE_CONFIG_SET_MOTION
#define TX_TYPE_CONFIG_SET_MOTION_RESPONSE SERIAL_TYPE_CONFIG_SET_MOTION_RESPONSE
#define RX_TYPE_TELEMETRY_REQUEST SERIAL_TYPE_TELEMETRY_REQUEST
#define RX_TYPE_ROBOT_VELOCITY SERIAL_TYPE_ROBOT_VELOCITY
#define ROBOT_VELOCITY_PROTOCOL_VERSION SERIAL_ROBOT_VELOCITY_PROTOCOL_VERSION
#define TX_TYPE_TELEMETRY_RESPONSE SERIAL_TYPE_TELEMETRY_RESPONSE
#define RX_TYPE_DISCOVERY_REQUEST SERIAL_TYPE_DISCOVERY_REQUEST
#define TX_TYPE_DISCOVERY_RESPONSE SERIAL_TYPE_DISCOVERY_RESPONSE
#define TELEMETRY_PROTOCOL_VERSION SERIAL_TELEMETRY_PROTOCOL_VERSION
#define DISCOVERY_PROTOCOL_VERSION SERIAL_DISCOVERY_PROTOCOL_VERSION
#define TELEMETRY_FLAG_BASIC (1U << 0)
#define TELEMETRY_FLAG_MOTORS (1U << 1)
#define TELEMETRY_FLAG_BATTERY (1U << 2)
#define TELEMETRY_FLAG_DIAGNOSTICS (1U << 3)
#define TELEMETRY_FLAG_FULL 0x0FU
#define TELEMETRY_MIN_INTERVAL_MS SERIAL_TELEMETRY_MIN_INTERVAL_MS
#define TELEMETRY_TURNAROUND_MS SERIAL_TELEMETRY_TURNAROUND_MS
#define ROBOT_ID_BROADCAST SERIAL_ROBOT_ID_BROADCAST
#define ROBOT_CONFIG_KEY 0x46434449U
#define ROBOT_UID_LEN ROBOT_IDENTITY_UID_LEN

#define Crc16CcittFalse SerialProtocol_Crc16
#define U16LE SerialProtocol_ReadU16LE
#define U32LE SerialProtocol_ReadU32LE
#define I16LE SerialProtocol_ReadI16LE
#define WriteU16LE SerialProtocol_WriteU16LE
#define WriteU32LE SerialProtocol_WriteU32LE
#define WriteI16LE SerialProtocol_WriteI16LE
#define SaturateI16 SerialProtocol_SaturateI16
#define RobotIdIsValid RobotIdentity_IsValidId
#define RobotUidRead RobotIdentity_ReadUid
#define RobotConfig_SetId RobotIdentity_SetId
#define RobotConfigGeneration RobotIdentity_Generation

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
static uint8_t discovery_response_pending = 0U;
static uint16_t discovery_request_sequence = 0U;
static uint32_t discovery_response_due_tick = 0U;
static uint8_t telemetry_has_sent = 0U;
static uint32_t config_response_due_tick = 0U;
static uint8_t config_response_pending = 0U;
static uint8_t config_response_type = 0U;
static uint8_t config_response_status = 0U;

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

static UART_HandleTypeDef *serial_uart = NULL;
static DMA_HandleTypeDef *serial_rx_dma = NULL;
static CommMode serial_comm_mode = (CommMode)TAURA_COMM_MODE;
static CrsfParser crsf_parser;

static void Serial_ProcessRx(void);
static void Serial_ProcessByte(uint8_t b);
static void Serial_ProcessBenchByte(uint8_t b);
static void Serial_ProcessMatchByte(uint8_t b);
static void Serial_ProcessCommandPacket(const uint8_t *buf);
static void Serial_TelemetryTask(void);
static void Serial_DiscoveryTask(void);
static void Serial_ConfigResponseTask(void);
static uint8_t Serial_QueueTx(const uint8_t *data, uint16_t len, uint8_t high_prio);
static void Serial_TxKick(void);
static void Serial_UartRecoveryTask(void);
static void Serial_ProcessDiscoverPacket(const uint8_t *buf);
static void Serial_ProcessSetIdPacket(const uint8_t *buf);
static void Serial_ProcessSetMotionPacket(const uint8_t *buf);
static void Serial_ProcessTelemetryRequest(const uint8_t *buf);
static void Serial_ProcessDiscoveryRequest(const uint8_t *buf);

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

static void Serial_ProcessRobotVelocityPacket(const uint8_t *buf)
{
  const uint16_t crc_offset = ROBOT_VELOCITY_PACKET_LEN - 2U;
  const uint16_t crc_rx = U16LE(&buf[crc_offset]);
  const uint16_t crc_ok = Crc16CcittFalse(buf, crc_offset);
  serial_dbg_crc_rx = crc_rx;
  serial_dbg_crc_calc = crc_ok;
  if ((crc_rx != crc_ok) || (buf[3] != ROBOT_VELOCITY_PROTOCOL_VERSION))
  {
    serial_dbg_rx_bad_crc++;
    return;
  }

  serial_dbg_rx_frames++;
  const uint8_t robot_id = buf[4];
  if ((live_robot_configured == 0U) ||
      ((robot_id != live_robot_id) && (robot_id != ROBOT_ID_BROADCAST)))
  {
    return;
  }

  const uint32_t sequence = U32LE(&buf[5]);
  const float vx = (float)I16LE(&buf[9]) * 0.001f;
  const float vy = (float)I16LE(&buf[11]) * 0.001f;
  const float omega = (float)I16LE(&buf[13]) * 0.001f;
  const uint8_t kick_power = buf[15];
  const uint8_t brake_mode = buf[16];
  serial_dbg_rx_vx = vx;
  serial_dbg_rx_vy = vy;
  serial_dbg_rx_omega = omega;
  AppC_SetRobotVelocity(sequence, vx, vy, omega, kick_power, brake_mode);
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

  return (HAL_UART_Transmit(serial_uart, response, sizeof(response), 40U) == HAL_OK) ? 1U : 0U;
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

static void Serial_ProcessSetMotionPacket(const uint8_t *buf)
{
  const uint16_t crc_rx = U16LE(&buf[CONFIG_SET_MOTION_PACKET_LEN - 2U]);
  const uint16_t crc_ok = Crc16CcittFalse(buf, CONFIG_SET_MOTION_PACKET_LEN - 2U);
  uint8_t uid[ROBOT_UID_LEN];
  RobotUidRead(uid);
  if ((crc_rx != crc_ok) || (memcmp(&buf[3], uid, ROBOT_UID_LEN) != 0) ||
      (U32LE(&buf[23]) != ROBOT_CONFIG_KEY))
  {
    return;
  }

  float max_linear_accel = 0.0f;
  float max_angular_accel = 0.0f;
  memcpy(&max_linear_accel, &buf[15], sizeof(max_linear_accel));
  memcpy(&max_angular_accel, &buf[19], sizeof(max_angular_accel));
  AppC_ForceSafeState();
  const uint8_t saved = RobotIdentity_SetMotionLimits(
      max_linear_accel, max_angular_accel);
  if (saved != 0U)
  {
    RobotMotionLimits limits;
    RobotIdentity_GetMotionLimits(&limits);
    AppC_SetMotionLimits(limits.max_linear_accel,
                         limits.max_angular_accel,
                         limits.max_brake_accel);
  }
  config_response_type = TX_TYPE_CONFIG_SET_MOTION_RESPONSE;
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
  if (crc_rx != crc_ok)
  {
    serial_dbg_rx_bad_crc++;
    return;
  }
  if ((buf[3] != TELEMETRY_PROTOCOL_VERSION) ||
      (live_robot_configured == 0U) ||
      (RobotIdIsValid(buf[4]) == 0U) ||
      (buf[4] != live_robot_id) ||
      (telemetry_response_pending != 0U) ||
      ((telemetry_has_sent != 0U) && ((now - last_telemetry_tick) < TELEMETRY_MIN_INTERVAL_MS)))
  {
    return;
  }

  telemetry_request_sequence = U16LE(&buf[5]);
  telemetry_request_flags = buf[7];
  serial_dbg_rx_frames++;
  telemetry_due_tick = now + TELEMETRY_TURNAROUND_MS;
  telemetry_response_pending = 1U;
}

static void Serial_ProcessDiscoveryRequest(const uint8_t *buf)
{
  const uint16_t crc_rx = U16LE(&buf[DISCOVERY_REQUEST_PACKET_LEN - 2U]);
  const uint16_t crc_ok = Crc16CcittFalse(buf, DISCOVERY_REQUEST_PACKET_LEN - 2U);
  if ((crc_rx != crc_ok) || (buf[3] != DISCOVERY_PROTOCOL_VERSION) ||
      (live_robot_configured == 0U))
  {
    return;
  }

  uint8_t uid[ROBOT_UID_LEN];
  RobotUidRead(uid);
  discovery_request_sequence = U16LE(&buf[4]);
  uint32_t slot_hash = (uint32_t)discovery_request_sequence ^ 0x7F4A7C15U;
  for (uint8_t i = 0U; i < ROBOT_UID_LEN; i++)
  {
    slot_hash = (slot_hash ^ uid[i]) * 0x85EBCA6BU;
  }
  discovery_response_due_tick = HAL_GetTick() + ((slot_hash & 0x0FU) * 20U);
  discovery_response_pending = 1U;
}

static void Serial_ProcessRx(void)
{
  uint16_t pos = UART_RX_DMA_BUF_SIZE - __HAL_DMA_GET_COUNTER(serial_uart->hdmarx);

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
  if (serial_comm_mode == COMM_MODE_MATCH)
  {
    Serial_ProcessMatchByte(b);
  }
  else
  {
    Serial_ProcessBenchByte(b);
  }
}

static void Serial_ProcessMatchByte(uint8_t b)
{
  serial_dbg_rx_bytes++;
  CrsfChannels channels;
  const CrsfParseResult result =
      CrsfParser_ProcessByte(&crsf_parser, b, &channels);
  if (result == CRSF_PARSE_RC_CHANNELS)
  {
    serial_dbg_rx_frames++;
    MatchControl_HandleChannels(&channels);
  }
  else if (result == CRSF_PARSE_BAD_CRC)
  {
    serial_dbg_rx_bad_crc++;
  }
}

static void Serial_ProcessBenchByte(uint8_t b)
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
        else if (frame[2] == RX_TYPE_DISCOVERY_REQUEST)
        {
          expected_len = DISCOVERY_REQUEST_PACKET_LEN;
        }
        else if (frame[2] == RX_TYPE_CONFIG_SET_MOTION)
        {
          expected_len = CONFIG_SET_MOTION_PACKET_LEN;
        }
        else if (frame[2] == RX_TYPE_ROBOT_VELOCITY)
        {
          expected_len = ROBOT_VELOCITY_PACKET_LEN;
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
        else if (frame[2] == RX_TYPE_DISCOVERY_REQUEST)
        {
          Serial_ProcessDiscoveryRequest(frame);
        }
        else if (frame[2] == RX_TYPE_CONFIG_SET_MOTION)
        {
          Serial_ProcessSetMotionPacket(frame);
        }
        else if (frame[2] == RX_TYPE_ROBOT_VELOCITY)
        {
          Serial_ProcessRobotVelocityPacket(frame);
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

  const uint32_t primask = __get_PRIMASK();
  __disable_irq();
  if (uart_tx_q_count >= UART_TX_QUEUE_DEPTH)
  {
    if (primask == 0U)
    {
      __enable_irq();
    }
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

  const uint32_t primask = __get_PRIMASK();
  __disable_irq();
  if ((uart_tx_busy != 0U) || (uart_tx_q_count == 0U))
  {
    if (primask == 0U)
    {
      __enable_irq();
    }
    return;
  }

  const uint8_t idx = uart_tx_q_head;
  const uint16_t len = uart_tx_queue_len[idx];
  memcpy(uart_tx_buf, uart_tx_queue[idx], len);
  uart_tx_q_head = (uart_tx_q_head + 1U) % UART_TX_QUEUE_DEPTH;
  uart_tx_q_count--;
  uart_tx_busy = 1U;
  if (primask == 0U)
  {
    __enable_irq();
  }

  if (HAL_UART_Transmit_DMA(serial_uart, uart_tx_buf, len) != HAL_OK)
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

  const uint32_t primask = __get_PRIMASK();
  __disable_irq();
  uart_recovery_pending = 0U;
  uart_tx_busy = 0U;
  uart_tx_q_head = 0U;
  uart_tx_q_tail = 0U;
  uart_tx_q_count = 0U;
  telemetry_response_pending = 0U;
  discovery_response_pending = 0U;
  if (primask == 0U)
  {
    __enable_irq();
  }

  (void)HAL_UART_Abort(serial_uart);
  __HAL_UART_CLEAR_OREFLAG(serial_uart);
  uart_rx_last_pos = 0U;
  if (HAL_UART_Receive_DMA(serial_uart, uart_rx_dma_buf, UART_RX_DMA_BUF_SIZE) == HAL_OK)
  {
    __HAL_DMA_DISABLE_IT(serial_rx_dma, DMA_IT_HT);
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
  uint8_t frame[TELEMETRY_RESPONSE_PACKET_LEN_MAX] = {0U};
  uint16_t len = 0U;
  const uint8_t flags = telemetry_request_flags & TELEMETRY_FLAG_FULL;
  frame[len++] = RX_SOF0;
  frame[len++] = RX_SOF1;
  frame[len++] = TX_TYPE_TELEMETRY_RESPONSE;
  frame[len++] = TELEMETRY_PROTOCOL_VERSION;
  frame[len++] = live_robot_id;
  WriteU16LE(&frame[len], telemetry_request_sequence);
  len += 2U;
  frame[len++] = (uint8_t)(1U | (uint8_t)(telem.fault_status << 1));
  frame[len++] = flags;

  if ((flags & TELEMETRY_FLAG_BASIC) != 0U)
  {
    WriteU32LE(&frame[len], telem.time_ms); len += 4U;
    frame[len++] = telem.communication_ok;
    frame[len++] = telem.stop_mode_brake;
    frame[len++] = telem.kick_power;
  }
  if ((flags & TELEMETRY_FLAG_MOTORS) != 0U)
  {
    WriteI16LE(&frame[len], SaturateI16(telem.rpm_m1 * 10.0f)); len += 2U;
    WriteI16LE(&frame[len], SaturateI16(telem.rpm_m2 * 10.0f)); len += 2U;
    WriteI16LE(&frame[len], SaturateI16(telem.rpm_m3 * 10.0f)); len += 2U;
    WriteI16LE(&frame[len], SaturateI16(telem.rpm_m4 * 10.0f)); len += 2U;
    WriteI16LE(&frame[len], SaturateI16((float)telem.cmd_m1)); len += 2U;
    WriteI16LE(&frame[len], SaturateI16((float)telem.cmd_m2)); len += 2U;
    WriteI16LE(&frame[len], SaturateI16((float)telem.cmd_m3)); len += 2U;
    WriteI16LE(&frame[len], SaturateI16((float)telem.cmd_m4)); len += 2U;
  }
  if ((flags & TELEMETRY_FLAG_BATTERY) != 0U)
  {
    WriteU16LE(&frame[len], (uint16_t)(telem.battery_voltage_v * 1000.0f));
    len += 2U;
    WriteU16LE(&frame[len], (uint16_t)telem.battery_adc_raw);
    len += 2U;
  }
  if ((flags & TELEMETRY_FLAG_DIAGNOSTICS) != 0U)
  {
    WriteU32LE(&frame[len], serial_dbg_rx_bad_crc); len += 4U;
    WriteU32LE(&frame[len], serial_dbg_rx_frames); len += 4U;
    frame[len++] = telem.communication_ok;
    WriteU32LE(&frame[len], telem.last_command_sequence); len += 4U;
  }

  const uint16_t crc = Crc16CcittFalse(frame, len);
  WriteU16LE(&frame[len], crc);
  len += 2U;

  if (Serial_QueueTx(frame, len, 0U) != 0U)
  {
    last_telemetry_tick = now;
    telemetry_has_sent = 1U;
    telemetry_response_pending = 0U;
  }
}

static void Serial_DiscoveryTask(void)
{
  if ((discovery_response_pending == 0U) ||
      ((int32_t)(HAL_GetTick() - discovery_response_due_tick) < 0))
  {
    return;
  }

  AppC_Telemetry telem;
  AppC_GetTelemetry(&telem);
  uint8_t frame[DISCOVERY_RESPONSE_PACKET_LEN] = {0U};
  uint8_t uid[ROBOT_UID_LEN];
  RobotUidRead(uid);
  frame[0] = RX_SOF0;
  frame[1] = RX_SOF1;
  frame[2] = TX_TYPE_DISCOVERY_RESPONSE;
  frame[3] = DISCOVERY_PROTOCOL_VERSION;
  frame[4] = live_robot_id;
  WriteU16LE(&frame[5], discovery_request_sequence);
  memcpy(&frame[7], uid, ROBOT_UID_LEN);
  frame[19] = SERIAL_FIRMWARE_VERSION_MAJOR;
  frame[20] = SERIAL_FIRMWARE_VERSION_MINOR;
  frame[21] = SERIAL_FIRMWARE_VERSION_PATCH;
  frame[22] = (uint8_t)(1U | (uint8_t)(telem.fault_status << 1));
  WriteU16LE(&frame[23], (uint16_t)(telem.battery_voltage_v * 1000.0f));
  WriteU16LE(&frame[25], Crc16CcittFalse(frame, 25U));
  if (Serial_QueueTx(frame, sizeof(frame), 0U) != 0U)
  {
    discovery_response_pending = 0U;
  }
}

HAL_StatusTypeDef SerialService_Init(UART_HandleTypeDef *uart,
                                     DMA_HandleTypeDef *rx_dma)
{
  serial_uart = uart;
  serial_rx_dma = rx_dma;
  CrsfParser_Init(&crsf_parser);
  MatchControl_Init();
  const HAL_StatusTypeDef status =
      HAL_UART_Receive_DMA(serial_uart, uart_rx_dma_buf, UART_RX_DMA_BUF_SIZE);
  if (status != HAL_OK)
  {
    return status;
  }
  __HAL_DMA_DISABLE_IT(serial_rx_dma, DMA_IT_HT);
  if (serial_comm_mode == COMM_MODE_BENCH)
  {
    (void)HAL_UART_Transmit(serial_uart, (uint8_t *)"USART2 READY\r\n",
                            strlen("USART2 READY\r\n"), 20U);
  }
  return HAL_OK;
}

void SerialService_Task(void)
{
  Serial_UartRecoveryTask();
  Serial_ProcessRx();
  if (serial_comm_mode == COMM_MODE_MATCH)
  {
    MatchControl_Task();
  }
  else
  {
    Serial_ConfigResponseTask();
    Serial_DiscoveryTask();
    Serial_TelemetryTask();
  }
}

void SerialService_OnTxComplete(UART_HandleTypeDef *uart)
{
  if (uart == serial_uart)
  {
    uart_tx_busy = 0U;
    Serial_TxKick();
  }
}

void SerialService_OnError(UART_HandleTypeDef *uart)
{
  if (uart == serial_uart)
  {
    uart_error_count++;
    uart_tx_busy = 0U;
    uart_recovery_pending = 1U;
  }
}

void SerialService_SetCommMode(CommMode mode)
{
  serial_comm_mode = mode;
  CrsfParser_Init(&crsf_parser);
  MatchControl_Init();
  telemetry_response_pending = 0U;
  discovery_response_pending = 0U;
  config_response_pending = 0U;
}

CommMode SerialService_GetCommMode(void)
{
  return serial_comm_mode;
}
