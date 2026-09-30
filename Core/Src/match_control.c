#include "match_control.h"

#include "app_c_api.h"
#include "robot_identity.h"
#include <string.h>

#define MATCH_ENABLE_THRESHOLD CRSF_RC_CHANNEL_CENTER
#define MATCH_ACTION_BITS 4U

static uint32_t match_sequence = 0U;
static uint32_t last_command_tick = 0U;
static uint8_t has_match_command = 0U;
static uint8_t last_action_bits[3] = {0U, 0U, 0U};
static uint8_t last_action_valid[3] = {0U, 0U, 0U};
static MatchControlStats stats;

static uint8_t DecodeUnsignedBits(uint16_t channel, uint8_t bits)
{
  const uint16_t clamped = (channel < CRSF_RC_CHANNEL_MIN) ? CRSF_RC_CHANNEL_MIN :
      ((channel > CRSF_RC_CHANNEL_MAX) ? CRSF_RC_CHANNEL_MAX : channel);
  const uint16_t levels = (uint16_t)((1U << bits) - 1U);
  const uint32_t scaled = ((uint32_t)(clamped - CRSF_RC_CHANNEL_MIN) * levels) +
                          ((CRSF_RC_CHANNEL_MAX - CRSF_RC_CHANNEL_MIN) / 2U);
  return (uint8_t)(scaled / (CRSF_RC_CHANNEL_MAX - CRSF_RC_CHANNEL_MIN));
}

static uint8_t DecodeKickPower(uint16_t channel)
{
  const uint16_t clamped = (channel < CRSF_RC_CHANNEL_MIN) ? CRSF_RC_CHANNEL_MIN :
      ((channel > CRSF_RC_CHANNEL_MAX) ? CRSF_RC_CHANNEL_MAX : channel);
  const uint32_t scaled = ((uint32_t)(clamped - CRSF_RC_CHANNEL_MIN) * 100U) +
                          ((CRSF_RC_CHANNEL_MAX - CRSF_RC_CHANNEL_MIN) / 2U);
  return (uint8_t)(scaled / (CRSF_RC_CHANNEL_MAX - CRSF_RC_CHANNEL_MIN));
}

static int8_t RobotIndex(uint8_t robot_id)
{
  if ((robot_id >= (uint8_t)'A') && (robot_id <= (uint8_t)'C'))
  {
    return (int8_t)(robot_id - (uint8_t)'A');
  }
  return -1;
}

uint8_t MatchControl_DecodeChannelsForRobot(const uint16_t channels[CRSF_RC_CHANNEL_COUNT],
                                            uint8_t robot_id,
                                            uint32_t sequence,
                                            RobotCommand *command)
{
  if ((channels == 0) || (command == 0))
  {
    return 0U;
  }

  const int8_t idx = RobotIndex(robot_id);
  if (idx < 0)
  {
    return 0U;
  }

  const uint8_t base = (uint8_t)idx * 4U;
  const uint8_t extra = 12U + (uint8_t)idx;
  const uint8_t enabled = (channels[15] > MATCH_ENABLE_THRESHOLD) ? 1U : 0U;
  const uint8_t action_bits = DecodeUnsignedBits(channels[base + 3U], MATCH_ACTION_BITS);
  if (enabled == 0U)
  {
    last_action_bits[(uint8_t)idx] = action_bits;
    last_action_valid[(uint8_t)idx] = 1U;
    RobotCommand_MakeSafe(sequence, command);
    return 1U;
  }

  const uint8_t previous_bits = last_action_bits[(uint8_t)idx];
  const uint8_t previous_valid = last_action_valid[(uint8_t)idx];
  last_action_bits[(uint8_t)idx] = action_bits;
  last_action_valid[(uint8_t)idx] = 1U;

  command->vx = Crsf_DequantizeSigned(channels[base], MATCH_MAX_VX);
  command->vy = Crsf_DequantizeSigned(channels[base + 1U], MATCH_MAX_VY);
  command->omega = Crsf_DequantizeSigned(channels[base + 2U], MATCH_MAX_OMEGA);
  command->kick = (previous_valid != 0U) &&
                  ((action_bits & ROBOT_COMMAND_ACTION_KICK) != 0U) &&
                  ((previous_bits & ROBOT_COMMAND_ACTION_KICK) == 0U);
  command->chip = ((action_bits & ROBOT_COMMAND_ACTION_CHIP) != 0U) ? 1U : 0U;
  command->brake = ((action_bits & ROBOT_COMMAND_ACTION_BRAKE) != 0U) ? 1U : 0U;
  command->dribbler = ((action_bits & ROBOT_COMMAND_ACTION_DRIBBLER) != 0U) ? 1U : 0U;
  command->kick_power = command->kick ? DecodeKickPower(channels[extra]) : 0U;
  command->enabled = 1U;
  command->sequence = sequence;
  return 1U;
}

void MatchControl_Init(void)
{
  match_sequence = 0U;
  last_command_tick = 0U;
  has_match_command = 0U;
  memset(last_action_bits, 0, sizeof(last_action_bits));
  memset(last_action_valid, 0, sizeof(last_action_valid));
  memset(&stats, 0, sizeof(stats));
}

void MatchControl_HandleChannels(const CrsfChannels *channels)
{
  if ((channels == 0) || (live_robot_configured == 0U))
  {
    stats.ignored_frames++;
    return;
  }

  RobotCommand command;
  match_sequence++;
  if (MatchControl_DecodeChannelsForRobot(channels->channels, live_robot_id,
                                          match_sequence, &command) == 0U)
  {
    stats.ignored_frames++;
    return;
  }

  last_command_tick = HAL_GetTick();
  has_match_command = 1U;
  stats.accepted_frames++;
  stats.last_sequence = command.sequence;
  AppC_ApplyRobotCommand(&command);
}

void MatchControl_Task(void)
{
  if ((has_match_command != 0U) &&
      ((HAL_GetTick() - last_command_tick) >= MATCH_COMMAND_TIMEOUT_MS))
  {
    has_match_command = 0U;
    stats.watchdog_trips++;
    AppC_ForceSafeState();
  }
}

void MatchControl_GetStats(MatchControlStats *out)
{
  if (out != 0)
  {
    *out = stats;
  }
}
