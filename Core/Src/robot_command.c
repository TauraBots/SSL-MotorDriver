#include "robot_command.h"

void RobotCommand_MakeSafe(uint32_t sequence, RobotCommand *command)
{
  if (command == 0)
  {
    return;
  }

  command->vx = 0.0f;
  command->vy = 0.0f;
  command->omega = 0.0f;
  command->kick_power = 0U;
  command->kick = 0U;
  command->chip = 0U;
  command->brake = 1U;
  command->dribbler = 0U;
  command->enabled = 0U;
  command->sequence = sequence;
}

