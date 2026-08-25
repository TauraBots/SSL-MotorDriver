#pragma once

#include <stdint.h>

#define ROBOT_IDENTITY_UID_LEN 12U

extern volatile uint8_t live_robot_id;
extern volatile uint8_t live_robot_configured;
extern volatile uint32_t live_robot_uid_word0;
extern volatile uint32_t live_robot_uid_word1;
extern volatile uint32_t live_robot_uid_word2;

typedef struct
{
  float max_linear_accel;
  float max_angular_accel;
  float max_brake_accel;
} RobotMotionLimits;

void RobotIdentity_Init(void);
uint8_t RobotIdentity_IsValidId(uint8_t robot_id);
uint8_t RobotIdentity_SetId(uint8_t robot_id);
void RobotIdentity_ReadUid(uint8_t uid[ROBOT_IDENTITY_UID_LEN]);
uint32_t RobotIdentity_Generation(void);
void RobotIdentity_GetMotionLimits(RobotMotionLimits *limits);
uint8_t RobotIdentity_SetMotionLimits(float max_linear_accel,
                                      float max_angular_accel);
