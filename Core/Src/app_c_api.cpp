#include "app_c_api.h"

#include "app.hpp"
#include "omni_kinematics.hpp"
#include <cmath>

extern "C"
{
volatile uint32_t live_comm_last_sequence = 0U;
volatile uint32_t live_comm_age_ms = 0U;
volatile uint32_t live_comm_accepted_packets = 0U;
volatile uint32_t live_comm_rejected_packets = 0U;
volatile uint32_t live_comm_duplicate_packets = 0U;
volatile uint32_t live_comm_stale_packets = 0U;
volatile uint8_t live_comm_state = 0U;
volatile uint8_t live_comm_kick_power = 0U;
}

namespace
{
App *g_app = nullptr;
constexpr float kSetpointMaxRpm = 530.0f;
constexpr float kWheelRadiusM = 0.03f;
constexpr float kRobotRadiusM = 0.09f;
constexpr float kRadSToRpm = 9.5492966f;
constexpr uint32_t kCommunicationTimeoutMs = 150U;
uint32_t g_lastCommandSequence = 0U;
uint32_t g_lastSequenceTick = 0U;
uint8_t g_hasCommand = 0U;
uint8_t g_communicationOk = 0U;
uint8_t g_kickPower = 0U;
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

inline bool IsNewerSequence(uint32_t sequence, uint32_t reference)
{
  const uint32_t delta = sequence - reference;
  return (delta != 0U) && (delta < 0x80000000U);
}

inline bool IsAcceptableSequence(uint32_t sequence)
{
  if (g_hasCommand == 0U)
  {
    return true;
  }

  if (g_communicationOk == 0U)
  {
    // Allow a restarted transmitter to establish a new sequence epoch after
    // timeout, while a frozen packet with the last sequence remains rejected.
    return sequence != g_lastCommandSequence;
  }

  return IsNewerSequence(sequence, g_lastCommandSequence);
}

void EnterSafeState()
{
  const uint32_t primask = __get_PRIMASK();
  __disable_irq();
  setpoint_m1 = 0.0f;
  setpoint_m2 = 0.0f;
  setpoint_m3 = 0.0f;
  setpoint_m4 = 0.0f;
  stop_mode_brake = 1U;
  g_kickPower = 0U;
  g_communicationOk = 0U;
  live_comm_state = 0U;
  live_comm_kick_power = 0U;
  if (g_app != nullptr)
  {
    g_app->ResetPidStates();
  }
  if (primask == 0U)
  {
    __enable_irq();
  }
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
  EnterSafeState();
}

extern "C" void AppC_Tick(void)
{
  if (g_app != nullptr)
  {
    g_app->Tick();
  }
  if ((g_hasCommand != 0U) && (g_communicationOk != 0U) &&
      ((HAL_GetTick() - g_lastSequenceTick) >= kCommunicationTimeoutMs))
  {
    EnterSafeState();
  }
  live_comm_age_ms = (g_hasCommand != 0U) ? (HAL_GetTick() - g_lastSequenceTick) : 0U;
}

extern "C" void AppC_FastTick1kHz(void)
{
  if (g_app != nullptr)
  {
    g_app->FastTick1kHz();
  }
}

extern "C" void AppC_SetCommands(uint32_t sequence,
                                  float m1, float m2, float m3, float m4,
                                  uint8_t kick_power, uint8_t brake_mode)
{
  const float newSetpointM1 = ClampSetpoint(m1);
  const float newSetpointM2 = ClampSetpoint(m2);
  const float newSetpointM3 = ClampSetpoint(m3);
  const float newSetpointM4 = ClampSetpoint(m4);
  const uint8_t newBrakeMode = (brake_mode != 0U) ? 1U : 0U;
  const uint8_t newKickPower = (kick_power > 100U) ? 100U : kick_power;

  const uint32_t primask = __get_PRIMASK();
  __disable_irq();

  if (!IsAcceptableSequence(sequence))
  {
    live_comm_rejected_packets++;
    if (sequence == g_lastCommandSequence)
    {
      live_comm_duplicate_packets++;
    }
    else
    {
      live_comm_stale_packets++;
    }
    if (primask == 0U)
    {
      __enable_irq();
    }
    return;
  }

  g_lastCommandSequence = sequence;
  g_lastSequenceTick = HAL_GetTick();
  g_hasCommand = 1U;
  g_communicationOk = 1U;
  live_comm_last_sequence = sequence;
  live_comm_age_ms = 0U;
  live_comm_accepted_packets++;
  live_comm_state = 1U;
  setpoint_m1 = newSetpointM1;
  setpoint_m2 = newSetpointM2;
  setpoint_m3 = newSetpointM3;
  setpoint_m4 = newSetpointM4;
  //cmd_m1 = static_cast<int32_t>(std::lround(setpoint_m1 * 10.0f));
  //cmd_m2 = static_cast<int32_t>(std::lround(setpoint_m2 * 10.0f));
  //cmd_m3 = static_cast<int32_t>(std::lround(setpoint_m3 * 10.0f));
  //cmd_m4 = static_cast<int32_t>(std::lround(setpoint_m4 * 10.0f));
  stop_mode_brake = newBrakeMode;
  g_kickPower = newKickPower;
  live_comm_kick_power = newKickPower;

  if (primask == 0U)
  {
    __enable_irq();
  }
}

extern "C" void AppC_SetRobotVelocity(uint32_t sequence,
                                        float vx, float vy, float omega,
                                        uint8_t kick_power, uint8_t brake_mode)
{
  static const OmniKinematics kinematics(kWheelRadiusM, kRobotRadiusM);
  float wheelRadS[4] = {0.0f, 0.0f, 0.0f, 0.0f};
  kinematics.RobotToWheels(vx, vy, omega, wheelRadS);
  AppC_SetCommands(sequence,
                   wheelRadS[0] * kRadSToRpm,
                   wheelRadS[1] * kRadSToRpm,
                   wheelRadS[2] * kRadSToRpm,
                   wheelRadS[3] * kRadSToRpm,
                   kick_power, brake_mode);
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
  out->last_command_sequence = g_lastCommandSequence;
  out->communication_ok = g_communicationOk;
  out->kick_power = g_kickPower;
  out->rpm_m1 = rpm_m1;
  out->rpm_m2 = rpm_m2;
  out->rpm_m3 = rpm_m3;
  out->rpm_m4 = rpm_m4;
  out->battery_adc_raw = battery_adc_raw;
  out->battery_voltage_v = battery_voltage_v;
}

extern "C" void AppC_ForceSafeState(void)
{
  EnterSafeState();
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
