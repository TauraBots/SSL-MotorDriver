#include "match_control.h"

#include "app_c_api.h"
#include "robot_identity.h"
#include <string.h>

#define MATCH_ENABLE_THRESHOLD CRSF_RC_CHANNEL_CENTER
#define MATCH_ACTION_BITS 4U

static uint32_t match_sequence = 0U;
static uint32_t last_command_tick = 0U;
static uint32_t command_timeout_ms = MATCH_COMMAND_TIMEOUT_MS;
static uint8_t has_match_command = 0U;
static uint8_t last_command_is_team = 0U;
static uint8_t last_action_bits[3] = {0U, 0U, 0U};
static uint8_t last_action_valid[3] = {0U, 0U, 0U};
static uint8_t last_team_flags[3] = {0U, 0U, 0U};
static uint8_t last_team_flags_valid[3] = {0U, 0U, 0U};
static uint16_t last_team_sequence = 0U;
static uint32_t expanded_team_sequence = 0U;
static uint8_t has_team_sequence = 0U;
static uint32_t last_valid_team_frame_tick = 0U;
static uint8_t has_valid_team_frame_tick = 0U;
static MatchControlStats stats;

volatile uint32_t match_dbg_team_frames_ok = 0U;
volatile uint32_t match_dbg_team_frames_bad_crc = 0U;
volatile uint32_t match_dbg_team_frames_duplicate = 0U;
volatile uint32_t match_dbg_team_frames_old = 0U;
volatile uint32_t match_dbg_team_frames_missed = 0U;
volatile uint32_t match_dbg_watchdog_trips = 0U;
volatile uint32_t match_dbg_team_timeouts = 0U;
volatile uint32_t match_dbg_last_interframe_ms = 0U;
volatile uint32_t match_dbg_max_interframe_ms = 0U;
volatile uint32_t match_dbg_last_sequence = 0U;

typedef enum
{
  TEAM_SEQUENCE_NEW = 0,
  TEAM_SEQUENCE_DUPLICATE,
  TEAM_SEQUENCE_OLD
} TeamSequenceStatus;

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

static TeamSequenceStatus AcceptTeamSequence(uint16_t sequence,
                                             uint32_t *expanded_sequence)
{
  if (has_team_sequence == 0U)
  {
    has_team_sequence = 1U;
    last_team_sequence = sequence;
    expanded_team_sequence = sequence;
    *expanded_sequence = expanded_team_sequence;
    return TEAM_SEQUENCE_NEW;
  }

  const uint16_t delta = (uint16_t)(sequence - last_team_sequence);
  if (delta == 0U)
  {
    return TEAM_SEQUENCE_DUPLICATE;
  }
  if (delta >= 0x8000U)
  {
    if (has_match_command != 0U)
    {
      return TEAM_SEQUENCE_OLD;
    }
    /* After a watchdog timeout, permit a restarted host to establish a new
     * sequence epoch without classifying that restart as packet loss. */
    last_team_sequence = sequence;
    expanded_team_sequence++;
    *expanded_sequence = expanded_team_sequence;
    return TEAM_SEQUENCE_NEW;
  }

  if (delta > 1U)
  {
    match_dbg_team_frames_missed += (uint32_t)(delta - 1U);
  }

  last_team_sequence = sequence;
  expanded_team_sequence += delta;
  *expanded_sequence = expanded_team_sequence;
  return TEAM_SEQUENCE_NEW;
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
  last_command_is_team = 0U;
  command_timeout_ms = MATCH_COMMAND_TIMEOUT_MS;
  memset(last_action_bits, 0, sizeof(last_action_bits));
  memset(last_action_valid, 0, sizeof(last_action_valid));
  memset(last_team_flags, 0, sizeof(last_team_flags));
  memset(last_team_flags_valid, 0, sizeof(last_team_flags_valid));
  last_team_sequence = 0U;
  expanded_team_sequence = 0U;
  has_team_sequence = 0U;
  last_valid_team_frame_tick = 0U;
  has_valid_team_frame_tick = 0U;
  memset(&stats, 0, sizeof(stats));
  match_dbg_team_frames_ok = 0U;
  match_dbg_team_frames_bad_crc = 0U;
  match_dbg_team_frames_duplicate = 0U;
  match_dbg_team_frames_old = 0U;
  match_dbg_team_frames_missed = 0U;
  match_dbg_watchdog_trips = 0U;
  match_dbg_team_timeouts = 0U;
  match_dbg_last_interframe_ms = 0U;
  match_dbg_max_interframe_ms = 0U;
  match_dbg_last_sequence = 0U;
  AppC_ForceSafeState();
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
  command_timeout_ms = MATCH_COMMAND_TIMEOUT_MS;
  has_match_command = 1U;
  last_command_is_team = 0U;
  stats.accepted_frames++;
  stats.last_sequence = command.sequence;
  AppC_ApplyRobotCommand(&command);
}

void MatchControl_HandleTeamFrame(const uint8_t *frame, uint16_t len)
{
  SerialTeamVelocityFrame decoded;
  const SerialTeamDecodeResult decode_result =
      SerialProtocol_DecodeTeamVelocity(frame, len, &decoded);
  if (decode_result != SERIAL_TEAM_DECODE_OK)
  {
    if (decode_result == SERIAL_TEAM_DECODE_BAD_CRC)
    {
      match_dbg_team_frames_bad_crc++;
    }
    else if (decode_result == SERIAL_TEAM_DECODE_BAD_VERSION)
    {
      stats.team_frames_bad_version++;
    }
    stats.ignored_frames++;
    return;
  }

  if (live_robot_configured == 0U)
  {
    stats.ignored_frames++;
    return;
  }

  const int8_t robot_index = RobotIndex(live_robot_id);
  if (robot_index < 0)
  {
    stats.ignored_frames++;
    return;
  }

  uint32_t command_sequence = 0U;
  const TeamSequenceStatus sequence_status =
      AcceptTeamSequence(decoded.sequence, &command_sequence);
  if (sequence_status == TEAM_SEQUENCE_DUPLICATE)
  {
    match_dbg_team_frames_duplicate++;
    stats.ignored_frames++;
    return;
  }
  if (sequence_status == TEAM_SEQUENCE_OLD)
  {
    match_dbg_team_frames_old++;
    stats.ignored_frames++;
    return;
  }

  const uint32_t now = HAL_GetTick();
  if (has_valid_team_frame_tick != 0U)
  {
    const uint32_t gap = now - last_valid_team_frame_tick;
    match_dbg_last_interframe_ms = gap;
    if (gap > match_dbg_max_interframe_ms)
    {
      match_dbg_max_interframe_ms = gap;
    }
  }
  last_valid_team_frame_tick = now;
  has_valid_team_frame_tick = 1U;
  match_dbg_last_sequence = decoded.sequence;

  const uint8_t index = (uint8_t)robot_index;
  const SerialTeamRobotSlot *slot = &decoded.robots[index];
  const uint8_t previous_flags = last_team_flags[index];
  const uint8_t previous_flags_valid = last_team_flags_valid[index];
  last_team_flags[index] = slot->flags;
  last_team_flags_valid[index] = 1U;

  RobotCommand command;
  if ((slot->flags & SERIAL_TEAM_FLAG_ENABLED) == 0U)
  {
    RobotCommand_MakeSafe(command_sequence, &command);
  }
  else
  {
    command.vx = (float)slot->vx_milli * 0.001f;
    command.vy = (float)slot->vy_milli * 0.001f;
    command.omega = (float)slot->omega_milli * 0.001f;
    command.kick = (previous_flags_valid != 0U) &&
                   ((slot->flags & SERIAL_TEAM_FLAG_KICK) != 0U) &&
                   ((previous_flags & SERIAL_TEAM_FLAG_KICK) == 0U);
    command.chip = ((slot->flags & SERIAL_TEAM_FLAG_CHIP) != 0U) ? 1U : 0U;
    command.brake = ((slot->flags & SERIAL_TEAM_FLAG_BRAKE) != 0U) ? 1U : 0U;
    command.dribbler = ((slot->flags & SERIAL_TEAM_FLAG_DRIBBLER) != 0U) ? 1U : 0U;
    command.kick_power = (command.kick != 0U) ? slot->kick_power : 0U;
    command.enabled = 1U;
    command.sequence = command_sequence;
  }

  last_command_tick = now;
  command_timeout_ms = MATCH_AIRPORT_COMMAND_TIMEOUT_MS;
  has_match_command = 1U;
  last_command_is_team = 1U;
  stats.accepted_frames++;
  match_dbg_team_frames_ok++;
  stats.last_sequence = command_sequence;
  AppC_ApplyRobotCommand(&command);
}

void MatchControl_Task(void)
{
  if (has_match_command == 0U)
  {
    AppC_ForceSafeState();
    return;
  }

  if ((has_match_command != 0U) &&
      ((HAL_GetTick() - last_command_tick) >= command_timeout_ms))
  {
    has_match_command = 0U;
    match_dbg_watchdog_trips++;
    if (last_command_is_team != 0U)
    {
      match_dbg_team_timeouts++;
    }
    AppC_ForceSafeState();
  }
}

void MatchControl_GetStats(MatchControlStats *out)
{
  if (out != 0)
  {
    *out = stats;
    out->watchdog_trips = match_dbg_watchdog_trips;
    out->team_frames_ok = match_dbg_team_frames_ok;
    out->team_frames_bad_crc = match_dbg_team_frames_bad_crc;
    out->team_frames_duplicate = match_dbg_team_frames_duplicate;
    out->team_frames_old = match_dbg_team_frames_old;
    out->team_frames_missed = match_dbg_team_frames_missed;
    out->team_frames_timeout = match_dbg_team_timeouts;
    out->team_last_interframe_ms = match_dbg_last_interframe_ms;
    out->team_max_interframe_ms = match_dbg_max_interframe_ms;
  }
}
