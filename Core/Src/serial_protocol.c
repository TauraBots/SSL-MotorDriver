#include "serial_protocol.h"

#include <limits.h>

uint16_t SerialProtocol_Crc16(const uint8_t *data, uint16_t len)
{
  uint16_t crc = 0xFFFFU;
  for (uint16_t i = 0U; i < len; i++)
  {
    crc ^= (uint16_t)data[i] << 8;
    for (uint8_t bit = 0U; bit < 8U; bit++)
    {
      crc = ((crc & 0x8000U) != 0U) ?
          (uint16_t)((crc << 1) ^ 0x1021U) : (uint16_t)(crc << 1);
    }
  }
  return crc;
}

uint16_t SerialProtocol_ReadU16LE(const uint8_t *data)
{
  return (uint16_t)data[0] | ((uint16_t)data[1] << 8);
}

uint32_t SerialProtocol_ReadU32LE(const uint8_t *data)
{
  return (uint32_t)data[0] |
         ((uint32_t)data[1] << 8) |
         ((uint32_t)data[2] << 16) |
         ((uint32_t)data[3] << 24);
}

int16_t SerialProtocol_ReadI16LE(const uint8_t *data)
{
  return (int16_t)SerialProtocol_ReadU16LE(data);
}

void SerialProtocol_WriteU16LE(uint8_t *data, uint16_t value)
{
  data[0] = (uint8_t)value;
  data[1] = (uint8_t)(value >> 8);
}

void SerialProtocol_WriteU32LE(uint8_t *data, uint32_t value)
{
  data[0] = (uint8_t)value;
  data[1] = (uint8_t)(value >> 8);
  data[2] = (uint8_t)(value >> 16);
  data[3] = (uint8_t)(value >> 24);
}

void SerialProtocol_WriteI16LE(uint8_t *data, int16_t value)
{
  SerialProtocol_WriteU16LE(data, (uint16_t)value);
}

int16_t SerialProtocol_SaturateI16(float value)
{
  if (value > 32767.0f)
  {
    return INT16_MAX;
  }
  if (value < -32768.0f)
  {
    return INT16_MIN;
  }
  return (int16_t)value;
}

SerialTeamDecodeResult SerialProtocol_DecodeTeamVelocity(
    const uint8_t *data, uint16_t len, SerialTeamVelocityFrame *frame)
{
  if ((data == 0) || (frame == 0) ||
      (len != SERIAL_TEAM_VELOCITY_PACKET_LEN))
  {
    return SERIAL_TEAM_DECODE_BAD_LENGTH;
  }

  if ((data[0] != SERIAL_RX_SOF0) || (data[1] != SERIAL_RX_SOF1) ||
      (data[2] != SERIAL_TYPE_TEAM_VELOCITY))
  {
    return SERIAL_TEAM_DECODE_BAD_FORMAT;
  }

  const uint16_t crc_offset = SERIAL_TEAM_VELOCITY_PACKET_LEN - 2U;
  if (SerialProtocol_ReadU16LE(&data[crc_offset]) !=
      SerialProtocol_Crc16(data, crc_offset))
  {
    return SERIAL_TEAM_DECODE_BAD_CRC;
  }

  if (data[3] != SERIAL_TEAM_VELOCITY_PROTOCOL_VERSION)
  {
    return SERIAL_TEAM_DECODE_BAD_VERSION;
  }

  frame->sequence = SerialProtocol_ReadU16LE(&data[4]);
  for (uint8_t robot = 0U; robot < SERIAL_TEAM_ROBOT_COUNT; robot++)
  {
    const uint16_t offset = 6U + ((uint16_t)robot * SERIAL_TEAM_ROBOT_SLOT_LEN);
    frame->robots[robot].vx_milli = SerialProtocol_ReadI16LE(&data[offset]);
    frame->robots[robot].vy_milli = SerialProtocol_ReadI16LE(&data[offset + 2U]);
    frame->robots[robot].omega_milli = SerialProtocol_ReadI16LE(&data[offset + 4U]);
    frame->robots[robot].kick_power = data[offset + 6U];
    frame->robots[robot].flags = data[offset + 7U];
  }
  return SERIAL_TEAM_DECODE_OK;
}
