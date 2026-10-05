#pragma once

#include "crsf_protocol.h"
#include "robot_command.h"
#include "serial_protocol.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define MATCH_COMMAND_TIMEOUT_MS 50U
#define MATCH_AIRPORT_COMMAND_TIMEOUT_MS 120U
#define MATCH_MAX_VX 2.5f
#define MATCH_MAX_VY 2.5f
#define MATCH_MAX_OMEGA 8.0f

typedef struct
{
  uint32_t accepted_frames;
  uint32_t ignored_frames;
  uint32_t watchdog_trips;
  uint32_t last_sequence;
  uint32_t team_frames_ok;
  uint32_t team_frames_bad_crc;
  uint32_t team_frames_bad_version;
  uint32_t team_frames_duplicate;
  uint32_t team_frames_old;
  uint32_t team_frames_missed;
  uint32_t team_frames_timeout;
  uint32_t team_last_interframe_ms;
  uint32_t team_max_interframe_ms;
} MatchControlStats;

/* Live AirPort Team diagnostics intended for the STM32CubeIDE debugger. */
extern volatile uint32_t match_dbg_team_frames_ok;
extern volatile uint32_t match_dbg_team_frames_bad_crc;
extern volatile uint32_t match_dbg_team_frames_duplicate;
extern volatile uint32_t match_dbg_team_frames_old;
extern volatile uint32_t match_dbg_team_frames_missed;
extern volatile uint32_t match_dbg_watchdog_trips;
extern volatile uint32_t match_dbg_team_timeouts;
extern volatile uint32_t match_dbg_last_interframe_ms;
extern volatile uint32_t match_dbg_max_interframe_ms;
extern volatile uint32_t match_dbg_last_sequence;

uint8_t MatchControl_DecodeChannelsForRobot(const uint16_t channels[CRSF_RC_CHANNEL_COUNT],
                                            uint8_t robot_id,
                                            uint32_t sequence,
                                            RobotCommand *command);
void MatchControl_Init(void);
void MatchControl_HandleChannels(const CrsfChannels *channels);
void MatchControl_HandleTeamFrame(const uint8_t *frame, uint16_t len);
void MatchControl_Task(void);
void MatchControl_GetStats(MatchControlStats *stats);

#ifdef __cplusplus
}
#endif

