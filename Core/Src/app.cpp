#include "app.hpp"
#include <cmath>

namespace
{
constexpr float kEncPpr = 11.0f;
constexpr float kEncMult = 4.0f;
constexpr float kGearRatio = 18.8f;
constexpr float kTicksPerRevOut = kEncPpr * kEncMult * kGearRatio;
constexpr uint32_t kControlTickMs = 1U;
constexpr uint32_t kRpmWindowMs = 10U;
constexpr uint32_t kBatterySampleMs = 100U;
constexpr uint32_t kHeartbeatMs = 200U;
constexpr float kControlDtS = 0.001f;
constexpr float kPidKp = 17.6991f;
constexpr float kPidKi = 80.2681f;
constexpr float kPidKd = 0.001f;
constexpr float kPidOutMin = -2399.0f;
constexpr float kPidOutMax = 2399.0f;
constexpr float kZeroSetpointRpm = 3.0f;
constexpr float kMovingRpm = 3.0f;
constexpr float kStaticStartPwm[4] = {425.0f, 425.0f, 425.0f, 425.0f};
constexpr float kStaticRunPwm[4] = {40.0f, 40.0f, 40.0f, 40.0f};
constexpr float kEncoderFaultMaxMeasuredRpm = 2.0f;
constexpr int32_t kEncoderFaultMinPwm = 650;
constexpr uint16_t kEncoderFaultTripTicks = 300U;
constexpr float kLpB0 = 0.0036216815f;
constexpr float kLpB1 = 0.0072433630f;
constexpr float kLpB2 = 0.0036216815f;
constexpr float kLpA1 = -1.8226949f;
constexpr float kLpA2 = 0.83718165f;
constexpr float kAdcRefV = 3.3f;
constexpr float kAdcMax = 4095.0f;
constexpr float kBatteryDividerGain = (10.0f + 3.3f) / 3.3f;
constexpr float kBatteryUndervoltageTripV = 9.6f;
constexpr float kBatteryUndervoltageRecoverV = 10.2f;
constexpr uint8_t kBatteryFaultSamples = 3U;
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
volatile uint32_t battery_adc_raw = 0U;
volatile float battery_voltage_v = 0.0f;

volatile uint16_t dbg_fault_m1 = 0;
volatile uint16_t dbg_fault_m2 = 0;
volatile uint16_t dbg_fault_m3 = 0;
volatile uint16_t dbg_fault_m4 = 0;

volatile int32_t dbg_prefault_m1 = 0;
volatile int32_t dbg_prefault_m2 = 0;
volatile int32_t dbg_prefault_m3 = 0;
volatile int32_t dbg_prefault_m4 = 0;

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
      pid1_{kPidKp, kPidKi, kPidKd, 0.0f, 0.0f, kPidOutMin, kPidOutMax},
      pid2_{kPidKp, kPidKi, kPidKd, 0.0f, 0.0f, kPidOutMin, kPidOutMax},
      pid3_{kPidKp, kPidKi, kPidKd, 0.0f, 0.0f, kPidOutMin, kPidOutMax},
      pid4_{kPidKp, kPidKi, kPidKd, 0.0f, 0.0f, kPidOutMin, kPidOutMax},
      encoderFaultCount_{0U, 0U, 0U, 0U},
      batteryValid_(0U),
      batteryUndervoltage_(0U),
      batteryLowCount_(0U),
      batteryAdcFailureCount_(0U),
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
  encoderFaultCount_[0] = 0U;
  encoderFaultCount_[1] = 0U;
  encoderFaultCount_[2] = 0U;
  encoderFaultCount_[3] = 0U;

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
  e1_.UpdateRpm(kControlTickMs, kRpmWindowMs);
  e2_.UpdateRpm(kControlTickMs, kRpmWindowMs);
  e3_.UpdateRpm(kControlTickMs, kRpmWindowMs);
  e4_.UpdateRpm(kControlTickMs, kRpmWindowMs);

  rpm_m1 = LowPassStep(rpmFilt1_, e1_.Rpm());
  rpm_m2 = LowPassStep(rpmFilt2_, e2_.Rpm());
  rpm_m3 = LowPassStep(rpmFilt3_, e3_.Rpm());
  rpm_m4 = LowPassStep(rpmFilt4_, e4_.Rpm());
  ApplyMotors();
}

void App::ResetPidStates()
{
  ResetPi(pid1_);
  ResetPi(pid2_);
  ResetPi(pid3_);
  ResetPi(pid4_);
}

void App::ForceSafeOutputs()
{
  setpoint_m1 = setpoint_m2 = setpoint_m3 = setpoint_m4 = 0.0f;
  stop_mode_brake = 1U;
  ResetPidStates();
  m1_.ForceStop(true);
  m2_.ForceStop(true);
  m3_.ForceStop(true);
  m4_.ForceStop(true);
  cmd_m1 = cmd_m2 = cmd_m3 = cmd_m4 = 0;
}

uint8_t App::FaultStatus() const
{
  uint8_t status = 0U;
  for (uint8_t i = 0U; i < 4U; i++)
  {
    if (encoderFaultCount_[i] >= kEncoderFaultTripTicks)
    {
      status |= (uint8_t)(1U << i);
    }
  }
  if ((batteryValid_ == 0U) || (batteryUndervoltage_ != 0U))
  {
    status |= 0x10U;
  }
  return status;
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
    if (++batteryAdcFailureCount_ >= kBatteryFaultSamples)
    {
      batteryValid_ = 0U;
    }
    return;
  }

  if (HAL_ADC_PollForConversion(batteryAdc_, 2U) != HAL_OK)
  {
    HAL_ADC_Stop(batteryAdc_);
    if (++batteryAdcFailureCount_ >= kBatteryFaultSamples)
    {
      batteryValid_ = 0U;
    }
    return;
  }

  const uint32_t raw = HAL_ADC_GetValue(batteryAdc_);
  HAL_ADC_Stop(batteryAdc_);

  battery_adc_raw = raw;
  batteryAdcFailureCount_ = 0U;
  const float vAdc = (static_cast<float>(raw) * kAdcRefV) / kAdcMax;
  battery_voltage_v = vAdc * kBatteryDividerGain;
  batteryValid_ = 1U;
  if (battery_voltage_v < kBatteryUndervoltageTripV)
  {
    if (batteryLowCount_ < kBatteryFaultSamples)
    {
      batteryLowCount_++;
    }
    if (batteryLowCount_ >= kBatteryFaultSamples)
    {
      batteryUndervoltage_ = 1U;
    }
  }
  else if (battery_voltage_v >= kBatteryUndervoltageRecoverV)
  {
    batteryLowCount_ = 0U;
    batteryUndervoltage_ = 0U;
  }
  else if (batteryUndervoltage_ == 0U)
  {
    batteryLowCount_ = 0U;
  }
}

void App::ApplyMotors()
{
  const bool brakeMode = (stop_mode_brake != 0U);

  if ((batteryValid_ == 0U) || (batteryUndervoltage_ != 0U))
  {
    setpoint_m1 = setpoint_m2 = setpoint_m3 = setpoint_m4 = 0.0f;
    stop_mode_brake = 1U;
    ResetPidStates();
    m1_.ForceStop(true);
    m2_.ForceStop(true);
    m3_.ForceStop(true);
    m4_.ForceStop(true);
    cmd_m1 = cmd_m2 = cmd_m3 = cmd_m4 = 0;
    return;
  }

  const float piOut1 = ComputePid(pid1_, setpoint_m1, rpm_m1, kControlDtS);
  const float piOut2 = ComputePid(pid2_, setpoint_m2, rpm_m2, kControlDtS);
  const float piOut3 = ComputePid(pid3_, setpoint_m3, rpm_m3, kControlDtS);
  const float piOut4 = ComputePid(pid4_, setpoint_m4, rpm_m4, kControlDtS);
  int32_t out1 = static_cast<int32_t>(std::lround(ApplyStaticPwm(setpoint_m1, rpm_m1, piOut1, 0U)));
  int32_t out2 = static_cast<int32_t>(std::lround(ApplyStaticPwm(setpoint_m2, rpm_m2, piOut2, 1U)));
  int32_t out3 = static_cast<int32_t>(std::lround(ApplyStaticPwm(setpoint_m3, rpm_m3, piOut3, 2U)));
  int32_t out4 = static_cast<int32_t>(std::lround(ApplyStaticPwm(setpoint_m4, rpm_m4, piOut4, 3U)));

  dbg_prefault_m1 = out1;
  dbg_prefault_m2 = out2;
  dbg_prefault_m3 = out3;
  dbg_prefault_m4 = out4;

  out1 = ApplyEncoderFaultProtection(0U, pid1_, setpoint_m1, rpm_m1, out1);
  out2 = ApplyEncoderFaultProtection(1U, pid2_, setpoint_m2, rpm_m2, out2);
  out3 = ApplyEncoderFaultProtection(2U, pid3_, setpoint_m3, rpm_m3, out3);
  out4 = ApplyEncoderFaultProtection(3U, pid4_, setpoint_m4, rpm_m4, out4);

  cmd_m1 = out1;
  cmd_m2 = out2;
  cmd_m3 = out3;
  cmd_m4 = out4;

  dbg_fault_m1 = encoderFaultCount_[0];
  dbg_fault_m2 = encoderFaultCount_[1];
  dbg_fault_m3 = encoderFaultCount_[2];
  dbg_fault_m4 = encoderFaultCount_[3];

  m1_.ApplySigned(out1, brakeMode);
  m2_.ApplySigned(out2, brakeMode);
  m3_.ApplySigned(out3, brakeMode);
  m4_.ApplySigned(out4, brakeMode);
}

float App::ComputePid(PidState &pid,
                      float setpointRpm,
                      float measuredRpm,
                      float dtS)
{
  if (std::fabs(setpointRpm) < kZeroSetpointRpm)
  {
    pid.integral = 0.0f;
    pid.lastMeas = measuredRpm;
    return 0.0f;
  }

  const float error = setpointRpm - measuredRpm;
  const float dMeas = (measuredRpm - pid.lastMeas) / dtS;

  const float pTerm = pid.kp * error;
  const float dTerm = -pid.kd * dMeas;

  float integralCandidate =
      pid.integral + (pid.ki * error * dtS);

  if (integralCandidate > pid.outMax)
    integralCandidate = pid.outMax;
  else if (integralCandidate < pid.outMin)
    integralCandidate = pid.outMin;

  const float candidateOutput =
      pTerm + integralCandidate + dTerm;

  const bool saturatingHigh =
      (candidateOutput > pid.outMax) && (error > 0.0f);

  const bool saturatingLow =
      (candidateOutput < pid.outMin) && (error < 0.0f);

  if (!saturatingHigh && !saturatingLow)
  {
    pid.integral = integralCandidate;
  }

  float output = pTerm + pid.integral + dTerm;

  if (output > pid.outMax)
    output = pid.outMax;
  else if (output < pid.outMin)
    output = pid.outMin;

  pid.lastMeas = measuredRpm;

  return output;
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

  if (output > kPidOutMax)
  {
    output = kPidOutMax;
  }
  else if (output < kPidOutMin)
  {
    output = kPidOutMin;
  }

  return output;
}

void App::ResetPi(PidState &pid)
{
  pid.integral = 0.0f;
  pid.lastMeas = 0.0f;
}

int32_t App::ApplyEncoderFaultProtection(uint32_t motorIndex, PidState &pid, float setpointRpm, float measuredRpm, int32_t cmd)
{
  if (std::fabs(setpointRpm) < kZeroSetpointRpm)
  {
    encoderFaultCount_[motorIndex] = 0U;
    return 0;
  }

  const bool suspicious =
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
