#include "crsf_protocol.h"

#define CRSF_MIN_FRAME_LEN 4U
#define CRSF_MAX_FRAME_LEN 64U

static uint16_t Crsf_ClampChannel(uint16_t channel)
{
  if (channel < CRSF_RC_CHANNEL_MIN)
  {
    return CRSF_RC_CHANNEL_MIN;
  }
  if (channel > CRSF_RC_CHANNEL_MAX)
  {
    return CRSF_RC_CHANNEL_MAX;
  }
  return channel;
}

static uint8_t Crsf_IsValidAddress(uint8_t address)
{
  return ((address == CRSF_RX_TO_STM32_SYNC_ADDRESS) ||
          (address == CRSF_ADDRESS_CRSF_RECEIVER)) ? 1U : 0U;
}

uint8_t Crsf_Crc8DvbS2(const uint8_t *data, uint8_t len)
{
  uint8_t crc = 0U;
  for (uint8_t i = 0U; i < len; i++)
  {
    crc ^= data[i];
    for (uint8_t bit = 0U; bit < 8U; bit++)
    {
      crc = (crc & 0x80U) ? (uint8_t)((crc << 1) ^ 0xD5U) : (uint8_t)(crc << 1);
    }
  }
  return crc;
}

void CrsfParser_Init(CrsfParser *parser)
{
  if (parser == 0)
  {
    return;
  }
  parser->index = 0U;
  parser->expected_len = 0U;
}

CrsfParseResult CrsfParser_ProcessByte(CrsfParser *parser, uint8_t byte,
                                       CrsfChannels *channels)
{
  if ((parser == 0) || (channels == 0))
  {
    return CRSF_PARSE_NONE;
  }

  if (parser->index == 0U)
  {
    if (Crsf_IsValidAddress(byte) == 0U)
    {
      return CRSF_PARSE_NONE;
    }
    parser->buffer[parser->index++] = byte;
    return CRSF_PARSE_NONE;
  }

  parser->buffer[parser->index++] = byte;

  if (parser->index == 2U)
  {
    const uint8_t length = parser->buffer[1];
    if ((length < 2U) || ((uint16_t)length + 2U > CRSF_MAX_FRAME_LEN))
    {
      parser->index = 0U;
      parser->expected_len = 0U;
      return CRSF_PARSE_NONE;
    }
    parser->expected_len = (uint8_t)(length + 2U);
  }

  if ((parser->expected_len == 0U) || (parser->index < parser->expected_len))
  {
    return CRSF_PARSE_NONE;
  }

  const uint8_t frame_len = parser->expected_len;
  const uint8_t type = parser->buffer[2];
  const uint8_t crc_rx = parser->buffer[frame_len - 1U];
  const uint8_t crc_ok = Crsf_Crc8DvbS2(&parser->buffer[2], (uint8_t)(frame_len - 3U));
  parser->index = 0U;
  parser->expected_len = 0U;

  if (crc_rx != crc_ok)
  {
    return CRSF_PARSE_BAD_CRC;
  }

  if ((type == CRSF_TYPE_RC_CHANNELS_PACKED) &&
      (frame_len == CRSF_RC_FRAME_LEN))
  {
    Crsf_UnpackRcChannels(&parser->buffer[3], channels->channels);
    return CRSF_PARSE_RC_CHANNELS;
  }

  return CRSF_PARSE_NONE;
}

void Crsf_UnpackRcChannels(const uint8_t payload[CRSF_RC_CHANNEL_PAYLOAD_LEN],
                           uint16_t channels[CRSF_RC_CHANNEL_COUNT])
{
  uint32_t bit_buffer = 0U;
  uint8_t bits = 0U;
  uint8_t byte_index = 0U;

  for (uint8_t ch = 0U; ch < CRSF_RC_CHANNEL_COUNT; ch++)
  {
    while (bits < CRSF_RC_CHANNEL_VALUE_BITS)
    {
      bit_buffer |= ((uint32_t)payload[byte_index++]) << bits;
      bits = (uint8_t)(bits + 8U);
    }
    channels[ch] = (uint16_t)(bit_buffer & 0x07FFU);
    bit_buffer >>= CRSF_RC_CHANNEL_VALUE_BITS;
    bits = (uint8_t)(bits - CRSF_RC_CHANNEL_VALUE_BITS);
  }
}

void Crsf_PackRcChannels(const uint16_t channels[CRSF_RC_CHANNEL_COUNT],
                         uint8_t payload[CRSF_RC_CHANNEL_PAYLOAD_LEN])
{
  uint32_t bit_buffer = 0U;
  uint8_t bits = 0U;
  uint8_t byte_index = 0U;

  for (uint8_t i = 0U; i < CRSF_RC_CHANNEL_PAYLOAD_LEN; i++)
  {
    payload[i] = 0U;
  }

  for (uint8_t ch = 0U; ch < CRSF_RC_CHANNEL_COUNT; ch++)
  {
    bit_buffer |= ((uint32_t)(channels[ch] & 0x07FFU)) << bits;
    bits = (uint8_t)(bits + CRSF_RC_CHANNEL_VALUE_BITS);
    while (bits >= 8U)
    {
      payload[byte_index++] = (uint8_t)(bit_buffer & 0xFFU);
      bit_buffer >>= 8U;
      bits = (uint8_t)(bits - 8U);
    }
  }
}

uint16_t Crsf_QuantizeUnsigned(float value, float min_value, float max_value)
{
  if (max_value <= min_value)
  {
    return CRSF_RC_CHANNEL_MIN;
  }
  if (value < min_value)
  {
    value = min_value;
  }
  if (value > max_value)
  {
    value = max_value;
  }
  const float normalized = (value - min_value) / (max_value - min_value);
  const float wire = (float)CRSF_RC_CHANNEL_MIN +
                     normalized * (float)(CRSF_RC_CHANNEL_MAX - CRSF_RC_CHANNEL_MIN);
  return Crsf_ClampChannel((uint16_t)(wire + 0.5f));
}

uint16_t Crsf_QuantizeSigned(float value, float max_abs_value)
{
  if (max_abs_value <= 0.0f)
  {
    return CRSF_RC_CHANNEL_CENTER;
  }
  return Crsf_QuantizeUnsigned(value, -max_abs_value, max_abs_value);
}

float Crsf_DequantizeSigned(uint16_t channel, float max_abs_value)
{
  if (max_abs_value <= 0.0f)
  {
    return 0.0f;
  }
  const uint16_t clamped = Crsf_ClampChannel(channel);
  const float normalized = ((float)clamped - (float)CRSF_RC_CHANNEL_MIN) /
                           (float)(CRSF_RC_CHANNEL_MAX - CRSF_RC_CHANNEL_MIN);
  return ((normalized * 2.0f) - 1.0f) * max_abs_value;
}
