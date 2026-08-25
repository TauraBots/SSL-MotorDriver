#pragma once

#include "main.h"

#include <cstdint>

class Encoder
{
public:
  Encoder(TIM_HandleTypeDef *htim, float ticksPerRev, int8_t direction = 1);

  void Start();
  void UpdateRpm(uint32_t dtMs, uint32_t windowMs);

  float Rpm() const { return rpm_; }

private:
  static int32_t DeltaWithOverflow(int32_t now, int32_t prev, uint32_t arr);

  TIM_HandleTypeDef *htim_;
  float ticksPerRev_;
  int8_t direction_;
  int32_t lastCnt_;
  int32_t delta_;
  int32_t accumulatedDelta_;
  uint32_t accumulatedMs_;
  float rpm_;
};
