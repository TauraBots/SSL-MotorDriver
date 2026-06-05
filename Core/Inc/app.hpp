#pragma once

#include "encoder.hpp"
#include "motor.hpp"

#include <cstdint>

struct AppContext
{
  ADC_HandleTypeDef *batteryAdc;
  TIM_HandleTypeDef *pwmTim1;
  TIM_HandleTypeDef *pwmTim8;
  TIM_HandleTypeDef *encTim1;
  TIM_HandleTypeDef *encTim2;
  TIM_HandleTypeDef *encTim3;
  TIM_HandleTypeDef *encTim4;
  GPIO_TypeDef *ledPort;
  uint16_t ledPin;
};

class App
{
public:
  explicit App(const AppContext &ctx);

  void Init();
  void Tick();
  void FastTick1kHz();

private:
  struct PidState
  {
    float b0;
    float b1;
    float eLast;
    float uLast;
    float refFilt;
    float outMin;
    float outMax;
  };
  struct BiquadState
  {
    float x1;
    float x2;
    float y1;
    float y2;
  };
  void UpdateBattery();
  void ApplyMotors();
  float LowPassStep(BiquadState &f, float x);
  float ComputePiDiscrete(PidState &pid, float setpointRpm, float measuredRpm);
  void Heartbeat();

  Motor m1_;
  Motor m2_;
  Motor m3_;
  Motor m4_;

  Encoder e1_;
  Encoder e2_;
  Encoder e3_;
  Encoder e4_;

  ADC_HandleTypeDef *batteryAdc_;
  GPIO_TypeDef *ledPort_;
  uint16_t ledPin_;

  uint32_t lastBatteryTick_;
  uint32_t lastHeartbeatTick_;
  PidState pid1_;
  PidState pid2_;
  PidState pid3_;
  PidState pid4_;
  BiquadState rpmFilt1_;
  BiquadState rpmFilt2_;
  BiquadState rpmFilt3_;
  BiquadState rpmFilt4_;
};

extern volatile int32_t cmd_m1;
extern volatile int32_t cmd_m2;
extern volatile int32_t cmd_m3;
extern volatile int32_t cmd_m4;
extern volatile float setpoint_m1;
extern volatile float setpoint_m2;
extern volatile float setpoint_m3;
extern volatile float setpoint_m4;
extern volatile uint8_t stop_mode_brake;

extern volatile float rpm_m1;
extern volatile float rpm_m2;
extern volatile float rpm_m3;
extern volatile float rpm_m4;
extern volatile uint32_t battery_adc_raw;
extern volatile float battery_voltage_v;
