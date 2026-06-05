#include "app.hpp"
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
  const int32_t out1 = static_cast<int32_t>(std::lround(ComputePiDiscrete(pid1_, setpoint_m1, rpm_m1)));
  const int32_t out2 = static_cast<int32_t>(std::lround(ComputePiDiscrete(pid2_, setpoint_m2, rpm_m2)));
  const int32_t out3 = static_cast<int32_t>(std::lround(ComputePiDiscrete(pid3_, setpoint_m3, rpm_m3)));
  const int32_t out4 = static_cast<int32_t>(std::lround(ComputePiDiscrete(pid4_, setpoint_m4, rpm_m4)));

  m1_.ApplySigned(out1, brakeMode);
  m2_.ApplySigned(out2, brakeMode);
  m3_.ApplySigned(out3, brakeMode);
  m4_.ApplySigned(out4, brakeMode);
}

float App::ComputePiDiscrete(PidState &pid, float setpointRpm, float measuredRpm)
{
  if (std::fabs(setpointRpm) < 1.0f)
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
