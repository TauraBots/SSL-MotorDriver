#include "app.hpp"
#include "omni_kinematics.hpp"
#include <cmath>

namespace
{
constexpr float kEncPpr = 11.0f;
constexpr float kEncMult = 4.0f;
constexpr float kGearRatio = 18.8f;
constexpr float kTicksPerRevOut = kEncPpr * kEncMult * kGearRatio;
constexpr uint32_t kRpmSampleMs = 1U;
constexpr uint32_t kBatterySampleMs = 100U;
constexpr uint32_t kHeartbeatMs = 200U;
constexpr float kControlDtS = 0.001f;
constexpr float kPiB0 = 5.64705043f;
constexpr float kPiB1 = -5.62914013f;
constexpr float kPiOutMin = -2399.0f;
constexpr float kPiOutMax = 2399.0f;
constexpr float kRefTauS = 0.15f;
constexpr float kRefAlpha = kControlDtS / (kRefTauS + kControlDtS);
constexpr float kMaxWheelDeltaRpm = 10.0f;
constexpr float kZeroSetpointRpm = 3.0f;
constexpr float kMovingRpm = 3.0f;
constexpr float kStaticStartPwm[4] = {425.0f, 425.0f, 425.0f, 425.0f};
constexpr float kStaticRunPwm[4] = {40.0f, 40.0f, 40.0f, 40.0f};
constexpr float kEncoderFaultMinRefRpm = 20.0f;
constexpr float kEncoderFaultMaxMeasuredRpm = 2.0f;
constexpr int32_t kEncoderFaultMinPwm = 650;
constexpr uint16_t kEncoderFaultTripTicks = 300U;
constexpr float kWheelRadiusM = 0.03f;
constexpr float kRobotRadiusM = 0.09f;
constexpr float kRadpsToRpm = 9.5492966f;
constexpr float kLpB0 = 0.0036216815f;
constexpr float kLpB1 = 0.0072433630f;
constexpr float kLpB2 = 0.0036216815f;
constexpr float kLpA1 = -1.8226949f;
constexpr float kLpA2 = 0.83718165f;
constexpr float kAdcRefV = 3.3f;
constexpr float kAdcMax = 4095.0f;
constexpr float kBatteryDividerGain = (10.0f + 3.3f) / 3.3f;
}

volatile int32_t cmd_m1 = 0;
volatile int32_t cmd_m2 = 0;
volatile int32_t cmd_m3 = 0;
volatile int32_t cmd_m4 = 0;
volatile float setpoint_m1 = 0.0f;
volatile float setpoint_m2 = 0.0f;
volatile float setpoint_m3 = 0.0f;
volatile float setpoint_m4 = 0.0f;
volatile uint8_t stop_mode_brake = 0;

volatile float rpm_m1 = 0.0f;
volatile float rpm_m2 = 0.0f;
volatile float rpm_m3 = 0.0f;
volatile float rpm_m4 = 0.0f;
volatile float wheel_ref_ramped_m1 = 0.0f;
volatile float wheel_ref_ramped_m2 = 0.0f;
volatile float wheel_ref_ramped_m3 = 0.0f;
volatile float wheel_ref_ramped_m4 = 0.0f;
volatile float pi_ref_m1 = 0.0f;
volatile float pi_ref_m2 = 0.0f;
volatile float pi_ref_m3 = 0.0f;
volatile float pi_ref_m4 = 0.0f;
volatile float pi_error_m1 = 0.0f;
volatile float pi_error_m2 = 0.0f;
volatile float pi_error_m3 = 0.0f;
volatile float pi_error_m4 = 0.0f;
volatile float pi_out_m1 = 0.0f;
volatile float pi_out_m2 = 0.0f;
volatile float pi_out_m3 = 0.0f;
volatile float pi_out_m4 = 0.0f;
volatile uint8_t encoder_fault_m1 = 0U;
volatile uint8_t encoder_fault_m2 = 0U;
volatile uint8_t encoder_fault_m3 = 0U;
volatile uint8_t encoder_fault_m4 = 0U;
volatile uint16_t encoder_fault_count_m1 = 0U;
volatile uint16_t encoder_fault_count_m2 = 0U;
volatile uint16_t encoder_fault_count_m3 = 0U;
volatile uint16_t encoder_fault_count_m4 = 0U;
volatile uint8_t debug_open_loop_enable = 0U;
volatile int32_t debug_pwm_m1 = 0;
volatile int32_t debug_pwm_m2 = 0;
volatile int32_t debug_pwm_m3 = 0;
volatile int32_t debug_pwm_m4 = 0;
volatile uint8_t debug_robot_cmd_enable = 0U;
volatile float debug_vx_mps = 0.0f;
volatile float debug_vy_mps = 0.0f;
volatile float debug_vtheta_radps = 0.0f;
volatile uint32_t battery_adc_raw = 0U;
volatile float battery_voltage_v = 0.0f;

App::App(const AppContext &ctx): 
      /*Connect PWM channels*/

      m1_(ctx.pwmTim8, TIM_CHANNEL_1, TIM_CHANNEL_2),
      m2_(ctx.pwmTim8, TIM_CHANNEL_3, TIM_CHANNEL_4),
      m3_(ctx.pwmTim1, TIM_CHANNEL_1, TIM_CHANNEL_4),
      m4_(ctx.pwmTim1, TIM_CHANNEL_2, TIM_CHANNEL_3, true),

      /*Connect encoders. Note that M3 is reversed to match the physical orientation on the robot.*/
      e1_(ctx.encTim2, kTicksPerRevOut),
      e2_(ctx.encTim4, kTicksPerRevOut),
      e3_(ctx.encTim3, kTicksPerRevOut, -1),
      e4_(ctx.encTim1, kTicksPerRevOut),

      batteryAdc_(ctx.batteryAdc),
      ledPort_(ctx.ledPort),
      ledPin_(ctx.ledPin),
      lastBatteryTick_(0),
      lastHeartbeatTick_(0),
      pid1_{kPiB0, kPiB1, 0.0f, 0.0f, 0.0f, kPiOutMin, kPiOutMax},
      pid2_{kPiB0, kPiB1, 0.0f, 0.0f, 0.0f, kPiOutMin, kPiOutMax},
      pid3_{kPiB0, kPiB1, 0.0f, 0.0f, 0.0f, kPiOutMin, kPiOutMax},
      pid4_{kPiB0, kPiB1, 0.0f, 0.0f, 0.0f, kPiOutMin, kPiOutMax},
      wheelRefRamped_{0.0f, 0.0f, 0.0f, 0.0f},
      encoderFaultCount_{0U, 0U, 0U, 0U},
      encoderFaultLatched_{0U, 0U, 0U, 0U},
      rpmFilt1_{0.0f, 0.0f, 0.0f, 0.0f},
      rpmFilt2_{0.0f, 0.0f, 0.0f, 0.0f},
      rpmFilt3_{0.0f, 0.0f, 0.0f, 0.0f},
      rpmFilt4_{0.0f, 0.0f, 0.0f, 0.0f}
{
}

void App::Init()
{
  m1_.Start();
  m2_.Start();
  m3_.Start();
  m4_.Start();

  e1_.Start();
  e2_.Start();
  e3_.Start();
  e4_.Start();

  lastBatteryTick_ = HAL_GetTick();
  lastHeartbeatTick_ = HAL_GetTick();

  cmd_m1 = 0;
  cmd_m2 = 0;
  cmd_m3 = 0;
  cmd_m4 = 0;
  setpoint_m1 = 0.0f;
  setpoint_m2 = 0.0f;
  setpoint_m3 = 0.0f;
  setpoint_m4 = 0.0f;
  stop_mode_brake = 0;
  debug_open_loop_enable = 0U;
  debug_pwm_m1 = 0;
  debug_pwm_m2 = 0;
  debug_pwm_m3 = 0;
  debug_pwm_m4 = 0;
  debug_robot_cmd_enable = 0U;
  debug_vx_mps = 0.0f;
  debug_vy_mps = 0.0f;
  debug_vtheta_radps = 0.0f;
  wheelRefRamped_[0] = 0.0f;
  wheelRefRamped_[1] = 0.0f;
  wheelRefRamped_[2] = 0.0f;
  wheelRefRamped_[3] = 0.0f;
  wheel_ref_ramped_m1 = 0.0f;
  wheel_ref_ramped_m2 = 0.0f;
  wheel_ref_ramped_m3 = 0.0f;
  wheel_ref_ramped_m4 = 0.0f;
  encoderFaultCount_[0] = 0U;
  encoderFaultCount_[1] = 0U;
  encoderFaultCount_[2] = 0U;
  encoderFaultCount_[3] = 0U;
  encoderFaultLatched_[0] = 0U;
  encoderFaultLatched_[1] = 0U;
  encoderFaultLatched_[2] = 0U;
  encoderFaultLatched_[3] = 0U;
  encoder_fault_m1 = 0U;
  encoder_fault_m2 = 0U;
  encoder_fault_m3 = 0U;
  encoder_fault_m4 = 0U;
  encoder_fault_count_m1 = 0U;
  encoder_fault_count_m2 = 0U;
  encoder_fault_count_m3 = 0U;
  encoder_fault_count_m4 = 0U;

  if (batteryAdc_ != nullptr)
  {
    HAL_ADCEx_Calibration_Start(batteryAdc_);
  }
}

void App::Tick()
{
  UpdateBattery();
  Heartbeat();
}

void App::FastTick1kHz()
{
  e1_.UpdateRpm(kRpmSampleMs);
  e2_.UpdateRpm(kRpmSampleMs);
  e3_.UpdateRpm(kRpmSampleMs);
  e4_.UpdateRpm(kRpmSampleMs);

  rpm_m1 = LowPassStep(rpmFilt1_, e1_.Rpm());
  rpm_m2 = LowPassStep(rpmFilt2_, e2_.Rpm());
  rpm_m3 = LowPassStep(rpmFilt3_, e3_.Rpm());
  rpm_m4 = LowPassStep(rpmFilt4_, e4_.Rpm());
  ApplyMotors();
}

float App::LowPassStep(BiquadState &f, float x)
{
  const float y = (kLpB0 * x) + (kLpB1 * f.x1) + (kLpB2 * f.x2) - (kLpA1 * f.y1) - (kLpA2 * f.y2);
  f.x2 = f.x1;
  f.x1 = x;
  f.y2 = f.y1;
  f.y1 = y;
  return y;
}

void App::UpdateBattery()
{
  if (batteryAdc_ == nullptr)
  {
    return;
  }

  const uint32_t now = HAL_GetTick();
  if ((now - lastBatteryTick_) < kBatterySampleMs)
  {
    return;
  }

  lastBatteryTick_ = now;

  if (HAL_ADC_Start(batteryAdc_) != HAL_OK)
  {
    return;
  }

  if (HAL_ADC_PollForConversion(batteryAdc_, 2U) != HAL_OK)
  {
    HAL_ADC_Stop(batteryAdc_);
    return;
  }

  const uint32_t raw = HAL_ADC_GetValue(batteryAdc_);
  HAL_ADC_Stop(batteryAdc_);

  battery_adc_raw = raw;
  const float vAdc = (static_cast<float>(raw) * kAdcRefV) / kAdcMax;
  battery_voltage_v = vAdc * kBatteryDividerGain;
}

void App::ApplyMotors()
{
  const bool brakeMode = (stop_mode_brake != 0U);

  if (debug_open_loop_enable != 0U)
  {
    cmd_m1 = debug_pwm_m1;
    cmd_m2 = debug_pwm_m2;
    cmd_m3 = debug_pwm_m3;
    cmd_m4 = debug_pwm_m4;
    pi_out_m1 = static_cast<float>(debug_pwm_m1);
    pi_out_m2 = static_cast<float>(debug_pwm_m2);
    pi_out_m3 = static_cast<float>(debug_pwm_m3);
    pi_out_m4 = static_cast<float>(debug_pwm_m4);

    m1_.ApplySigned(debug_pwm_m1, brakeMode);
    m2_.ApplySigned(debug_pwm_m2, brakeMode);
    m3_.ApplySigned(debug_pwm_m3, brakeMode);
    m4_.ApplySigned(debug_pwm_m4, brakeMode);
    return;
  }

  if (debug_robot_cmd_enable != 0U)
  {
    OmniKinematics kin(kWheelRadiusM, kRobotRadiusM);
    float wheelRadps[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    kin.RobotToWheels(debug_vx_mps, debug_vy_mps, debug_vtheta_radps, wheelRadps);
    setpoint_m1 = wheelRadps[0] * kRadpsToRpm;
    setpoint_m2 = wheelRadps[1] * kRadpsToRpm;
    setpoint_m3 = wheelRadps[2] * kRadpsToRpm;
    setpoint_m4 = wheelRadps[3] * kRadpsToRpm;
  }

  wheelRefRamped_[0] = LimitRate(setpoint_m1, wheelRefRamped_[0], kMaxWheelDeltaRpm);
  wheelRefRamped_[1] = LimitRate(setpoint_m2, wheelRefRamped_[1], kMaxWheelDeltaRpm);
  wheelRefRamped_[2] = LimitRate(setpoint_m3, wheelRefRamped_[2], kMaxWheelDeltaRpm);
  wheelRefRamped_[3] = LimitRate(setpoint_m4, wheelRefRamped_[3], kMaxWheelDeltaRpm);
  wheel_ref_ramped_m1 = wheelRefRamped_[0];
  wheel_ref_ramped_m2 = wheelRefRamped_[1];
  wheel_ref_ramped_m3 = wheelRefRamped_[2];
  wheel_ref_ramped_m4 = wheelRefRamped_[3];

  const float piOut1 = ComputePiDiscrete(pid1_, wheelRefRamped_[0], rpm_m1);
  const float piOut2 = ComputePiDiscrete(pid2_, wheelRefRamped_[1], rpm_m2);
  const float piOut3 = ComputePiDiscrete(pid3_, wheelRefRamped_[2], rpm_m3);
  const float piOut4 = ComputePiDiscrete(pid4_, wheelRefRamped_[3], rpm_m4);
  int32_t out1 = static_cast<int32_t>(std::lround(ApplyStaticPwm(wheelRefRamped_[0], rpm_m1, piOut1, 0U)));
  int32_t out2 = static_cast<int32_t>(std::lround(ApplyStaticPwm(wheelRefRamped_[1], rpm_m2, piOut2, 1U)));
  int32_t out3 = static_cast<int32_t>(std::lround(ApplyStaticPwm(wheelRefRamped_[2], rpm_m3, piOut3, 2U)));
  int32_t out4 = static_cast<int32_t>(std::lround(ApplyStaticPwm(wheelRefRamped_[3], rpm_m4, piOut4, 3U)));
  out1 = ApplyEncoderFaultProtection(0U, pid1_, wheelRefRamped_[0], rpm_m1, out1);
  out2 = ApplyEncoderFaultProtection(1U, pid2_, wheelRefRamped_[1], rpm_m2, out2);
  out3 = ApplyEncoderFaultProtection(2U, pid3_, wheelRefRamped_[2], rpm_m3, out3);
  out4 = ApplyEncoderFaultProtection(3U, pid4_, wheelRefRamped_[3], rpm_m4, out4);
  encoder_fault_m1 = encoderFaultLatched_[0];
  encoder_fault_m2 = encoderFaultLatched_[1];
  encoder_fault_m3 = encoderFaultLatched_[2];
  encoder_fault_m4 = encoderFaultLatched_[3];
  encoder_fault_count_m1 = encoderFaultCount_[0];
  encoder_fault_count_m2 = encoderFaultCount_[1];
  encoder_fault_count_m3 = encoderFaultCount_[2];
  encoder_fault_count_m4 = encoderFaultCount_[3];

  pi_ref_m1 = pid1_.refFilt;
  pi_ref_m2 = pid2_.refFilt;
  pi_ref_m3 = pid3_.refFilt;
  pi_ref_m4 = pid4_.refFilt;
  pi_error_m1 = pid1_.eLast;
  pi_error_m2 = pid2_.eLast;
  pi_error_m3 = pid3_.eLast;
  pi_error_m4 = pid4_.eLast;
  pi_out_m1 = static_cast<float>(out1);
  pi_out_m2 = static_cast<float>(out2);
  pi_out_m3 = static_cast<float>(out3);
  pi_out_m4 = static_cast<float>(out4);

  cmd_m1 = out1;
  cmd_m2 = out2;
  cmd_m3 = out3;
  cmd_m4 = out4;

  m1_.ApplySigned(out1, brakeMode);
  m2_.ApplySigned(out2, brakeMode);
  m3_.ApplySigned(out3, brakeMode);
  m4_.ApplySigned(out4, brakeMode);
}

float App::LimitRate(float target, float current, float maxDelta) const
{
  const float diff = target - current;

  if (diff > maxDelta)
  {
    return current + maxDelta;
  }

  if (diff < -maxDelta)
  {
    return current - maxDelta;
  }

  return target;
}

float App::ComputePiDiscrete(PidState &pid, float setpointRpm, float measuredRpm)
{
  if (std::fabs(setpointRpm) < kZeroSetpointRpm)
  {
    pid.eLast = 0.0f;
    pid.uLast = 0.0f;
    pid.refFilt = 0.0f;
    return 0.0f;
  }

  pid.refFilt += kRefAlpha * (setpointRpm - pid.refFilt);

  const float error = pid.refFilt - measuredRpm;
  const float uUnsat = pid.uLast + (pid.b0 * error) + (pid.b1 * pid.eLast);
  float uSat = uUnsat;

  if (uSat > pid.outMax)
  {
    uSat = pid.outMax;
  }
  else if (uSat < pid.outMin)
  {
    uSat = pid.outMin;
  }

  if (std::fabs(uUnsat - uSat) < 1e-6f)
  {
    pid.uLast = uSat;
    pid.eLast = error;
  }

  return uSat;
}

float App::ApplyStaticPwm(float setpointRpm, float measuredRpm, float piOut, uint32_t motorIndex)
{
  if (std::fabs(setpointRpm) < kZeroSetpointRpm)
  {
    return 0.0f;
  }

  const float sign = (setpointRpm > 0.0f) ? 1.0f : -1.0f;
  const float staticPwm = (std::fabs(measuredRpm) < kMovingRpm) ? kStaticStartPwm[motorIndex] : kStaticRunPwm[motorIndex];
  float output = piOut + (sign * staticPwm);

  if (output > kPiOutMax)
  {
    output = kPiOutMax;
  }
  else if (output < kPiOutMin)
  {
    output = kPiOutMin;
  }

  return output;
}

void App::ResetPi(PidState &pid)
{
  pid.eLast = 0.0f;
  pid.uLast = 0.0f;
  pid.refFilt = 0.0f;
}

int32_t App::ApplyEncoderFaultProtection(uint32_t motorIndex, PidState &pid, float setpointRpm, float measuredRpm, int32_t cmd)
{
  if (std::fabs(setpointRpm) < kZeroSetpointRpm)
  {
    encoderFaultCount_[motorIndex] = 0U;
    encoderFaultLatched_[motorIndex] = 0U;
    return 0;
  }

  const bool suspicious =
      (std::fabs(setpointRpm) >= kEncoderFaultMinRefRpm) &&
      (std::abs(cmd) >= kEncoderFaultMinPwm) &&
      (std::fabs(measuredRpm) <= kEncoderFaultMaxMeasuredRpm);

  if (suspicious)
  {
    if (encoderFaultCount_[motorIndex] < kEncoderFaultTripTicks)
    {
      encoderFaultCount_[motorIndex]++;
    }
  }
  else
  {
    encoderFaultCount_[motorIndex] = 0U;
  }

  if (encoderFaultCount_[motorIndex] >= kEncoderFaultTripTicks)
  {
    encoderFaultLatched_[motorIndex] = 1U;
    ResetPi(pid);
    return 0;
  }

  return cmd;
}

void App::Heartbeat()
{
  const uint32_t now = HAL_GetTick();
  if ((now - lastHeartbeatTick_) < kHeartbeatMs)
  {
    return;
  }

  lastHeartbeatTick_ = now;
  HAL_GPIO_TogglePin(ledPort_, ledPin_);
}
