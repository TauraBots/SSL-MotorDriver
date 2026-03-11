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

void AppC_SetCommands(float m1, float m2, float m3, float m4, uint8_t brake_mode);
void AppC_GetTelemetry(AppC_Telemetry *out);
void AppC_RobotToWheels(float vx, float vy, float omega,
                        float wheel_radius_m, float robot_radius_m,
                        AppC_WheelSpeeds *out_wheels);
void AppC_WheelsToRobot(const AppC_WheelSpeeds *wheel_speeds,
                        float wheel_radius_m, float robot_radius_m,
                        float *out_vx, float *out_vy, float *out_omega);

#ifdef __cplusplus
}
#endif
