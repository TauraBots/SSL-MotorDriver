#include "robot_identity.h"

#include "main.h"
#include "robot_config.h"
#include "serial_protocol.h"
#include "stm32f1xx_hal_flash_ex.h"
#include <math.h>
#include <string.h>

#define ROBOT_CONFIG_ADDRESS 0x0803F800U
#define ROBOT_CONFIG_MAGIC 0x54425549U
#define ROBOT_CONFIG_VERSION 2U

typedef struct __attribute__((packed))
{
  uint32_t magic;
  uint16_t version;
  uint16_t size;
  uint8_t robot_id;
  uint8_t configured;
  uint8_t reserved[2];
  uint32_t generation;
  float max_linear_accel;
  float max_angular_accel;
  float max_brake_accel;
  uint32_t crc32;
} RobotIdentityConfig;

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
} RobotIdentityConfigV1;

typedef char RobotIdentityConfigSizeMustBe32Bytes[
    (sizeof(RobotIdentityConfig) == 32U) ? 1 : -1];

static RobotIdentityConfig live_config;

volatile uint8_t live_robot_id = 0U;
volatile uint8_t live_robot_configured = 0U;
volatile uint32_t live_robot_uid_word0 = 0U;
volatile uint32_t live_robot_uid_word1 = 0U;
volatile uint32_t live_robot_uid_word2 = 0U;

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

static void LoadDefaults(void)
{
  memset(&live_config, 0, sizeof(live_config));
  live_config.magic = ROBOT_CONFIG_MAGIC;
  live_config.version = ROBOT_CONFIG_VERSION;
  live_config.size = sizeof(live_config);
  live_config.max_linear_accel = ROBOT_MAX_LINEAR_ACCEL;
  live_config.max_angular_accel = ROBOT_MAX_ANGULAR_ACCEL;
  live_config.max_brake_accel = ROBOT_MAX_BRAKE_ACCEL;
}

static uint8_t MotionLimitsAreValid(float linear, float angular)
{
  return (isfinite(linear) && isfinite(angular) &&
          (linear >= 0.1f) && (linear <= 20.0f) &&
          (angular >= 0.1f) && (angular <= 50.0f)) ? 1U : 0U;
}

static uint8_t SaveConfig(void)
{
  live_config.generation++;
  live_config.crc32 = Crc32Ieee(
      (const uint8_t *)&live_config,
      (uint16_t)(sizeof(live_config) - sizeof(live_config.crc32)));
  FLASH_EraseInitTypeDef erase = {0};
  uint32_t page_error = 0U;
  erase.TypeErase = FLASH_TYPEERASE_PAGES;
  erase.PageAddress = ROBOT_CONFIG_ADDRESS;
  erase.NbPages = 1U;
  if ((HAL_FLASH_Unlock() != HAL_OK) ||
      (HAL_FLASHEx_Erase(&erase, &page_error) != HAL_OK))
  {
    (void)HAL_FLASH_Lock();
    return 0U;
  }
  for (uint32_t i = 0U; i < (sizeof(live_config) / sizeof(uint16_t)); i++)
  {
    const uint16_t halfword = SerialProtocol_ReadU16LE(
        &((const uint8_t *)&live_config)[i * sizeof(uint16_t)]);
    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD,
                         ROBOT_CONFIG_ADDRESS + (i * sizeof(uint16_t)),
                         halfword) != HAL_OK)
    {
      (void)HAL_FLASH_Lock();
      return 0U;
    }
  }
  (void)HAL_FLASH_Lock();
  RobotIdentityConfig verify;
  memcpy(&verify, (const void *)ROBOT_CONFIG_ADDRESS, sizeof(verify));
  return (memcmp(&verify, &live_config, sizeof(verify)) == 0) ? 1U : 0U;
}

uint8_t RobotIdentity_IsValidId(uint8_t robot_id)
{
  return ((robot_id >= (uint8_t)'A') && (robot_id <= (uint8_t)'Z')) ? 1U : 0U;
}

void RobotIdentity_ReadUid(uint8_t uid[ROBOT_IDENTITY_UID_LEN])
{
  memcpy(uid, (const void *)UID_BASE, ROBOT_IDENTITY_UID_LEN);
}

void RobotIdentity_Init(void)
{
  uint8_t uid[ROBOT_IDENTITY_UID_LEN];
  RobotIdentity_ReadUid(uid);
  live_robot_uid_word0 = SerialProtocol_ReadU32LE(&uid[0]);
  live_robot_uid_word1 = SerialProtocol_ReadU32LE(&uid[4]);
  live_robot_uid_word2 = SerialProtocol_ReadU32LE(&uid[8]);
  LoadDefaults();
  RobotIdentityConfig stored;
  memcpy(&stored, (const void *)ROBOT_CONFIG_ADDRESS, sizeof(stored));
  const uint32_t crc = Crc32Ieee(
      (const uint8_t *)&stored, (uint16_t)(sizeof(stored) - sizeof(stored.crc32)));
  if ((stored.magic == ROBOT_CONFIG_MAGIC) &&
      (stored.version == ROBOT_CONFIG_VERSION) &&
      (stored.size == sizeof(stored)) && (stored.crc32 == crc) &&
      (MotionLimitsAreValid(stored.max_linear_accel, stored.max_angular_accel) != 0U))
  {
    live_config = stored;
  }
  else
  {
    RobotIdentityConfigV1 old;
    memcpy(&old, (const void *)ROBOT_CONFIG_ADDRESS, sizeof(old));
    const uint32_t old_crc = Crc32Ieee(
        (const uint8_t *)&old, (uint16_t)(sizeof(old) - sizeof(old.crc32)));
    if ((old.magic == ROBOT_CONFIG_MAGIC) && (old.version == 1U) &&
        (old.size == sizeof(old)) && (old.crc32 == old_crc))
    {
      live_config.robot_id = old.robot_id;
      live_config.configured = old.configured;
      live_config.generation = old.generation;
    }
  }
  if ((live_config.configured == 1U) &&
      (RobotIdentity_IsValidId(live_config.robot_id) != 0U))
  {
    live_robot_id = live_config.robot_id;
    live_robot_configured = 1U;
  }
}

uint32_t RobotIdentity_Generation(void)
{
  return live_robot_configured ? live_config.generation : 0U;
}

void RobotIdentity_GetMotionLimits(RobotMotionLimits *limits)
{
  if (limits != NULL)
  {
    limits->max_linear_accel = live_config.max_linear_accel;
    limits->max_angular_accel = live_config.max_angular_accel;
    limits->max_brake_accel = live_config.max_brake_accel;
  }
}

uint8_t RobotIdentity_SetId(uint8_t robot_id)
{
  if (RobotIdentity_IsValidId(robot_id) == 0U)
  {
    return 0U;
  }
  live_config.robot_id = robot_id;
  live_config.configured = 1U;
  if (SaveConfig() == 0U)
  {
    return 0U;
  }
  live_robot_id = robot_id;
  live_robot_configured = 1U;
  return 1U;
}

uint8_t RobotIdentity_SetMotionLimits(float max_linear_accel,
                                      float max_angular_accel)
{
  if ((live_robot_configured == 0U) ||
      (MotionLimitsAreValid(max_linear_accel, max_angular_accel) == 0U))
  {
    return 0U;
  }
  const float old_linear = live_config.max_linear_accel;
  const float old_angular = live_config.max_angular_accel;
  live_config.max_linear_accel = max_linear_accel;
  live_config.max_angular_accel = max_angular_accel;
  if (SaveConfig() == 0U)
  {
    live_config.max_linear_accel = old_linear;
    live_config.max_angular_accel = old_angular;
    return 0U;
  }
  return 1U;
}
