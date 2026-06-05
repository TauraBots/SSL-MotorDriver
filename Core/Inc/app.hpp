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
  float LimitRate(float target, float current, float maxDelta) const;
  float ComputePiDiscrete(PidState &pid, float setpointRpm, float measuredRpm);
  float ApplyStaticPwm(float setpointRpm, float measuredRpm, float piOut, uint32_t motorIndex);
  void ResetPi(PidState &pid);
  int32_t ApplyEncoderFaultProtection(uint32_t motorIndex, PidState &pid, float setpointRpm, float measuredRpm, int32_t cmd);
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
  float wheelRefRamped_[4];
  uint16_t encoderFaultCount_[4];
  uint8_t encoderFaultLatched_[4];
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
extern volatile float wheel_ref_ramped_m1;
extern volatile float wheel_ref_ramped_m2;
extern volatile float wheel_ref_ramped_m3;
extern volatile float wheel_ref_ramped_m4;
extern volatile float pi_ref_m1;
extern volatile float pi_ref_m2;
extern volatile float pi_ref_m3;
extern volatile float pi_ref_m4;
extern volatile float pi_error_m1;
extern volatile float pi_error_m2;
extern volatile float pi_error_m3;
extern volatile float pi_error_m4;
extern volatile float pi_out_m1;
extern volatile float pi_out_m2;
extern volatile float pi_out_m3;
extern volatile float pi_out_m4;
extern volatile uint8_t encoder_fault_m1;
extern volatile uint8_t encoder_fault_m2;
extern volatile uint8_t encoder_fault_m3;
extern volatile uint8_t encoder_fault_m4;
extern volatile uint16_t encoder_fault_count_m1;
extern volatile uint16_t encoder_fault_count_m2;
extern volatile uint16_t encoder_fault_count_m3;
extern volatile uint16_t encoder_fault_count_m4;
extern volatile uint8_t debug_open_loop_enable;
extern volatile int32_t debug_pwm_m1;
extern volatile int32_t debug_pwm_m2;
extern volatile int32_t debug_pwm_m3;
extern volatile int32_t debug_pwm_m4;
extern volatile uint8_t debug_robot_cmd_enable;
extern volatile float debug_vx_mps;
extern volatile float debug_vy_mps;
extern volatile float debug_vtheta_radps;
extern volatile uint32_t battery_adc_raw;
extern volatile float battery_voltage_v;
