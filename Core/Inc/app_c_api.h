#pragma once

#include "main.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void AppC_Init(ADC_HandleTypeDef *batteryAdc,
               TIM_HandleTypeDef *pwmTim1,
               TIM_HandleTypeDef *pwmTim8,
               TIM_HandleTypeDef *encTim1,
               TIM_HandleTypeDef *encTim2,
               TIM_HandleTypeDef *encTim3,
               TIM_HandleTypeDef *encTim4,
               GPIO_TypeDef *ledPort,
               uint16_t ledPin);

void AppC_Tick(void);
void AppC_FastTick1kHz(void);

typedef struct
{
  uint32_t time_ms;
  int32_t cmd_m1;
  int32_t cmd_m2;
  int32_t cmd_m3;
  int32_t cmd_m4;
  uint8_t stop_mode_brake;
  uint32_t last_command_sequence;
  uint8_t communication_ok;
  uint8_t kick_power;
  uint8_t fault_status;
  float rpm_m1;
  float rpm_m2;
  float rpm_m3;
  float rpm_m4;
  uint32_t battery_adc_raw;
  float battery_voltage_v;
} AppC_Telemetry;

typedef struct
{
  float m1;
  float m2;
  float m3;
  float m4;
} AppC_WheelSpeeds;

/* Temporary debugger instrumentation. Angular values use rad/s; wheel values
 * and PID references/feedback use RPM. These fields do not affect control. */
extern volatile float dbg_yaw_omega_cmd_rad_s;
extern volatile float dbg_yaw_omega_limited_rad_s;
extern volatile float dbg_yaw_wheel_raw_rpm[4];
extern volatile float dbg_yaw_wheel_limited_rpm[4];
extern volatile float dbg_yaw_pid_reference_rpm[4];
extern volatile float dbg_yaw_feedback_rpm[4];

void AppC_SetCommands(uint32_t sequence,
                      float m1, float m2, float m3, float m4,
                      uint8_t kick_power, uint8_t brake_mode);
void AppC_SetRobotVelocity(uint32_t sequence,
                           float vx, float vy, float omega,
                           uint8_t kick_power, uint8_t brake_mode);
void AppC_GetTelemetry(AppC_Telemetry *out);
void AppC_ForceSafeState(void);
void AppC_SetMotionLimits(float max_linear_accel,
                          float max_angular_accel,
                          float max_brake_accel);
void AppC_EmergencyStop(void);
void AppC_RobotToWheels(float vx, float vy, float omega,
                        float wheel_radius_m, float robot_radius_m,
                        AppC_WheelSpeeds *out_wheels);
void AppC_WheelsToRobot(const AppC_WheelSpeeds *wheel_speeds,
                        float wheel_radius_m, float robot_radius_m,
                        float *out_vx, float *out_vy, float *out_omega);

#ifdef __cplusplus
}
#endif
