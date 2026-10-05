#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define ROBOT_COMMAND_ACTION_KICK      (1U << 0)
#define ROBOT_COMMAND_ACTION_CHIP      (1U << 1)
#define ROBOT_COMMAND_ACTION_BRAKE     (1U << 2)
#define ROBOT_COMMAND_ACTION_DRIBBLER  (1U << 3)

typedef struct
{
  float vx;
  float vy;
  float omega;
  uint8_t kick_power;
  uint8_t kick;
  uint8_t chip;
  uint8_t brake;
  uint8_t dribbler;
  uint8_t enabled;
  uint32_t sequence;
} RobotCommand;

void RobotCommand_MakeSafe(uint32_t sequence, RobotCommand *command);

#ifdef __cplusplus
}
#endif

