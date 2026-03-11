#pragma once

#include "main.h"

#include <cstdint>

class Motor
{
public:
  Motor(TIM_HandleTypeDef *htim, uint32_t channelA, uint32_t channelB, bool complementary = false);

  void Start() const;
  void ApplySigned(int32_t cmd, bool brakeMode) const;

private:
  static uint16_t ClampDuty(int32_t duty);
  void Stop(bool brakeMode) const;

  TIM_HandleTypeDef *htim_;
  uint32_t channelA_;
  uint32_t channelB_;
  bool complementary_;
};
