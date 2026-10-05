#pragma once

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define CRSF_SYNC_BYTE                         0xC8U
#define CRSF_ADDRESS_FLIGHT_CONTROLLER         0xC8U
#define CRSF_ADDRESS_CRSF_RECEIVER             0xECU
#define CRSF_ADDRESS_CRSF_TRANSMITTER          0xEEU
#define CRSF_RX_TO_STM32_SYNC_ADDRESS          CRSF_SYNC_BYTE
#define CRSF_HOST_TO_TX_MODULE_ADDRESS         CRSF_ADDRESS_CRSF_TRANSMITTER
#define CRSF_TYPE_RC_CHANNELS_PACKED   0x16U
#define CRSF_RC_CHANNEL_COUNT          16U
#define CRSF_RC_CHANNEL_MIN            172U
#define CRSF_RC_CHANNEL_CENTER         992U
#define CRSF_RC_CHANNEL_MAX            1811U
#define CRSF_RC_CHANNEL_VALUE_BITS     11U
#define CRSF_RC_CHANNEL_PAYLOAD_LEN    22U
#define CRSF_RC_FRAME_LEN              26U

typedef struct
{
  uint16_t channels[CRSF_RC_CHANNEL_COUNT];
} CrsfChannels;

typedef enum
{
  CRSF_PARSE_NONE = 0,
  CRSF_PARSE_RC_CHANNELS = 1,
  CRSF_PARSE_BAD_CRC = 2
} CrsfParseResult;

typedef struct
{
  uint8_t buffer[64];
  uint8_t index;
  uint8_t expected_len;
} CrsfParser;

void CrsfParser_Init(CrsfParser *parser);
CrsfParseResult CrsfParser_ProcessByte(CrsfParser *parser, uint8_t byte,
                                       CrsfChannels *channels);
uint8_t Crsf_Crc8DvbS2(const uint8_t *data, uint8_t len);
void Crsf_UnpackRcChannels(const uint8_t payload[CRSF_RC_CHANNEL_PAYLOAD_LEN],
                           uint16_t channels[CRSF_RC_CHANNEL_COUNT]);
void Crsf_PackRcChannels(const uint16_t channels[CRSF_RC_CHANNEL_COUNT],
                         uint8_t payload[CRSF_RC_CHANNEL_PAYLOAD_LEN]);
uint16_t Crsf_QuantizeUnsigned(float value, float min_value, float max_value);
float Crsf_DequantizeSigned(uint16_t channel, float max_abs_value);
uint16_t Crsf_QuantizeSigned(float value, float max_abs_value);

#ifdef __cplusplus
}
#endif
