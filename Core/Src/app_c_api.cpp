#include "app_c_api.h"

#include "app.hpp"
#include "acceleration_limiter.hpp"
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
volatile float dbg_yaw_omega_cmd_rad_s = 0.0f;
volatile float dbg_yaw_omega_limited_rad_s = 0.0f;
volatile float dbg_yaw_wheel_raw_rpm[4] = {0.0f, 0.0f, 0.0f, 0.0f};
volatile float dbg_yaw_wheel_limited_rpm[4] = {0.0f, 0.0f, 0.0f, 0.0f};
volatile float dbg_yaw_pid_reference_rpm[4] = {0.0f, 0.0f, 0.0f, 0.0f};
volatile float dbg_yaw_feedback_rpm[4] = {0.0f, 0.0f, 0.0f, 0.0f};
}

namespace
{
App *g_app = nullptr;
constexpr float kSetpointMaxRpm = 530.0f;
constexpr float kWheelRadiusM = 0.03f;
constexpr float kRobotRadiusM = 0.09f;
constexpr float kRadSToRpm = 9.5492966f;
constexpr float kControlDtS = 0.001f;
constexpr uint32_t kCommunicationTimeoutMs = 150U;
AccelerationLimiter g_accelerationLimiter(RobotMotionConfig());
OmniKinematics g_kinematics(kWheelRadiusM, kRobotRadiusM);
volatile float g_desiredVx = 0.0f;
volatile float g_desiredVy = 0.0f;
volatile float g_desiredOmega = 0.0f;
volatile uint8_t g_cartesianCommand = 0U;
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

bool AcceptCommandLocked(uint32_t sequence, uint8_t kickPower, uint8_t brakeMode)
{
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
    return false;
  }

  g_lastCommandSequence = sequence;
  g_lastSequenceTick = HAL_GetTick();
  g_hasCommand = 1U;
  g_communicationOk = 1U;
  g_kickPower = (kickPower > 100U) ? 100U : kickPower;
  stop_mode_brake = (brakeMode != 0U) ? 1U : 0U;
  live_comm_last_sequence = sequence;
  live_comm_age_ms = 0U;
  live_comm_accepted_packets++;
  live_comm_state = 1U;
  live_comm_kick_power = g_kickPower;
  return true;
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
  g_desiredVx = 0.0f;
  g_desiredVy = 0.0f;
  g_desiredOmega = 0.0f;
  g_accelerationLimiter.Reset();
  dbg_yaw_omega_cmd_rad_s = 0.0f;
  dbg_yaw_omega_limited_rad_s = 0.0f;
  for (uint8_t i = 0U; i < 4U; i++)
  {
    dbg_yaw_wheel_raw_rpm[i] = 0.0f;
    dbg_yaw_wheel_limited_rpm[i] = 0.0f;
    dbg_yaw_pid_reference_rpm[i] = 0.0f;
  }
  if (g_app != nullptr)
  {
    g_app->ForceSafeOutputs();
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
  if ((g_cartesianCommand != 0U) && (g_communicationOk != 0U))
  {
    const bool braking = (stop_mode_brake != 0U);
    const LimitedRobotVelocity applied = g_accelerationLimiter.Update(
        g_desiredVx, g_desiredVy, g_desiredOmega, kControlDtS, braking);
    float wheelRadS[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    float wheelRawRadS[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    g_kinematics.RobotToWheels(
        applied.vx, applied.vy, applied.omega, wheelRadS, wheelRawRadS);
    setpoint_m1 = ClampSetpoint(wheelRadS[0] * kRadSToRpm);
    setpoint_m2 = ClampSetpoint(wheelRadS[1] * kRadSToRpm);
    setpoint_m3 = ClampSetpoint(wheelRadS[2] * kRadSToRpm);
    setpoint_m4 = ClampSetpoint(wheelRadS[3] * kRadSToRpm);
    dbg_yaw_omega_cmd_rad_s = g_desiredOmega;
    dbg_yaw_omega_limited_rad_s = applied.omega;
    for (uint8_t i = 0U; i < 4U; i++)
    {
      dbg_yaw_wheel_raw_rpm[i] = wheelRawRadS[i] * kRadSToRpm;
      dbg_yaw_wheel_limited_rpm[i] = wheelRadS[i] * kRadSToRpm;
    }
    dbg_yaw_pid_reference_rpm[0] = setpoint_m1;
    dbg_yaw_pid_reference_rpm[1] = setpoint_m2;
    dbg_yaw_pid_reference_rpm[2] = setpoint_m3;
    dbg_yaw_pid_reference_rpm[3] = setpoint_m4;
  }
  if (g_app != nullptr)
  {
    g_app->FastTick1kHz();
  }
  dbg_yaw_feedback_rpm[0] = rpm_m1;
  dbg_yaw_feedback_rpm[1] = rpm_m2;
  dbg_yaw_feedback_rpm[2] = rpm_m3;
  dbg_yaw_feedback_rpm[3] = rpm_m4;
}

extern "C" void AppC_SetCommands(uint32_t sequence,
                                  float m1, float m2, float m3, float m4,
                                  uint8_t kick_power, uint8_t brake_mode)
{
  const float newSetpointM1 = ClampSetpoint(m1);
  const float newSetpointM2 = ClampSetpoint(m2);
  const float newSetpointM3 = ClampSetpoint(m3);
  const float newSetpointM4 = ClampSetpoint(m4);
  const uint32_t primask = __get_PRIMASK();
  __disable_irq();

  if (!AcceptCommandLocked(sequence, kick_power, brake_mode))
  {
    if (primask == 0U)
    {
      __enable_irq();
    }
    return;
  }

  g_cartesianCommand = 0U;
  g_accelerationLimiter.Reset();
  setpoint_m1 = newSetpointM1;
  setpoint_m2 = newSetpointM2;
  setpoint_m3 = newSetpointM3;
  setpoint_m4 = newSetpointM4;
  if (primask == 0U)
  {
    __enable_irq();
  }
}

extern "C" void AppC_SetRobotVelocity(uint32_t sequence,
                                        float vx, float vy, float omega,
                                        uint8_t kick_power, uint8_t brake_mode)
{
  RobotCommand command{};
  command.vx = vx;
  command.vy = vy;
  command.omega = omega;
  command.kick_power = kick_power;
  command.kick = (kick_power > 0U) ? 1U : 0U;
  command.chip = 0U;
  command.brake = brake_mode;
  command.dribbler = 0U;
  command.enabled = 1U;
  command.sequence = sequence;
  AppC_ApplyRobotCommand(&command);
}

extern "C" void AppC_ApplyRobotCommand(const RobotCommand *command)
{
  if (command == nullptr)
  {
    return;
  }

  if (command->enabled == 0U)
  {
    EnterSafeState();
    return;
  }

  const uint32_t primask = __get_PRIMASK();
  __disable_irq();
  const uint8_t kickPower = (command->kick != 0U) ? command->kick_power : 0U;
  if (AcceptCommandLocked(command->sequence, kickPower, command->brake))
  {
    g_desiredVx = command->vx;
    g_desiredVy = command->vy;
    g_desiredOmega = command->omega;
    g_cartesianCommand = 1U;
  }
  if (primask == 0U)
  {
    __enable_irq();
  }
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
  out->fault_status = (g_app != nullptr) ? g_app->FaultStatus() : 0U;
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

extern "C" void AppC_SetMotionLimits(float max_linear_accel,
                                      float max_angular_accel,
                                      float max_brake_accel)
{
  const MotionConfig defaults = RobotMotionConfig();
  const MotionConfig config = {
      max_linear_accel,
      max_angular_accel,
      max_brake_accel,
      defaults.maxLinearSpeed,
      defaults.maxAngularSpeed};
  const uint32_t primask = __get_PRIMASK();
  __disable_irq();
  g_accelerationLimiter.SetConfig(config);
  if (primask == 0U)
  {
    __enable_irq();
  }
}

extern "C" void AppC_EmergencyStop(void)
{
  TIM1->CCER = 0U;
  TIM8->CCER = 0U;
  TIM1->BDTR &= ~TIM_BDTR_MOE;
  TIM8->BDTR &= ~TIM_BDTR_MOE;
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
