#include "app_c_api.h"

#include "app.hpp"
#include "omni_kinematics.hpp"
#include <cmath>

namespace
{
App *g_app = nullptr;
constexpr float kSetpointMaxRpm = 530.0f;
inline float ClampSetpoint(float x)
{
  if (x > kSetpointMaxRpm)
  {
    return kSetpointMaxRpm;
  }
  if (x < -kSetpointMaxRpm)
  {
    return -kSetpointMaxRpm;
  }
  return x;
}
}

extern "C" void AppC_Init(ADC_HandleTypeDef *batteryAdc,
                            TIM_HandleTypeDef *pwmTim1,
                            TIM_HandleTypeDef *pwmTim8,
                            TIM_HandleTypeDef *encTim1,
                            TIM_HandleTypeDef *encTim2,
                            TIM_HandleTypeDef *encTim3,
                            TIM_HandleTypeDef *encTim4,
                            GPIO_TypeDef *ledPort,
                            uint16_t ledPin)
{
  static AppContext context = {
      batteryAdc,
      pwmTim1,
      pwmTim8,
      encTim1,
      encTim2,
      encTim3,
      encTim4,
      ledPort,
      ledPin};

  static App app(context);
  g_app = &app;
  g_app->Init();
}

extern "C" void AppC_Tick(void)
{
  if (g_app != nullptr)
  {
    g_app->Tick();
  }
}

extern "C" void AppC_FastTick1kHz(void)
{
  if (g_app != nullptr)
  {
    g_app->FastTick1kHz();
  }
}

extern "C" void AppC_SetCommands(float m1, float m2, float m3, float m4, uint8_t brake_mode)
{
  setpoint_m1 = ClampSetpoint(m1);
  setpoint_m2 = ClampSetpoint(m2);
  setpoint_m3 = ClampSetpoint(m3);
  setpoint_m4 = ClampSetpoint(m4);
  cmd_m1 = static_cast<int32_t>(std::lround(setpoint_m1 * 10.0f));
  cmd_m2 = static_cast<int32_t>(std::lround(setpoint_m2 * 10.0f));
  cmd_m3 = static_cast<int32_t>(std::lround(setpoint_m3 * 10.0f));
  cmd_m4 = static_cast<int32_t>(std::lround(setpoint_m4 * 10.0f));
  stop_mode_brake = (brake_mode != 0U) ? 1U : 0U;
}

extern "C" void AppC_GetTelemetry(AppC_Telemetry *out)
{
  if (out == nullptr)
  {
    return;
  }

  out->time_ms = HAL_GetTick();
  out->cmd_m1 = cmd_m1;
  out->cmd_m2 = cmd_m2;
  out->cmd_m3 = cmd_m3;
  out->cmd_m4 = cmd_m4;
  out->stop_mode_brake = stop_mode_brake;
  out->rpm_m1 = rpm_m1;
  out->rpm_m2 = rpm_m2;
  out->rpm_m3 = rpm_m3;
  out->rpm_m4 = rpm_m4;
  out->battery_adc_raw = battery_adc_raw;
  out->battery_voltage_v = battery_voltage_v;
}

extern "C" void AppC_RobotToWheels(float vx, float vy, float omega,
                                   float wheel_radius_m, float robot_radius_m,
                                   AppC_WheelSpeeds *out_wheels)
{
  if (out_wheels == nullptr)
  {
    return;
  }

  OmniKinematics kin(wheel_radius_m, robot_radius_m);
  float w[4] = {0.0f, 0.0f, 0.0f, 0.0f};
  kin.RobotToWheels(vx, vy, omega, w);
  out_wheels->m1 = w[0];
  out_wheels->m2 = w[1];
  out_wheels->m3 = w[2];
  out_wheels->m4 = w[3];
}

extern "C" void AppC_WheelsToRobot(const AppC_WheelSpeeds *wheel_speeds,
                                   float wheel_radius_m, float robot_radius_m,
                                   float *out_vx, float *out_vy, float *out_omega)
{
  if ((wheel_speeds == nullptr) || (out_vx == nullptr) || (out_vy == nullptr) || (out_omega == nullptr))
  {
    return;
  }

  OmniKinematics kin(wheel_radius_m, robot_radius_m);
  const float w[4] = {wheel_speeds->m1, wheel_speeds->m2, wheel_speeds->m3, wheel_speeds->m4};
  const OmniRobotTwist twist = kin.WheelsToRobot(w);
  *out_vx = twist.vx;
  *out_vy = twist.vy;
  *out_omega = twist.omega;
}
