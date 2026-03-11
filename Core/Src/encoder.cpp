#include "encoder.hpp"

Encoder::Encoder(TIM_HandleTypeDef *htim, float ticksPerRev, int8_t direction)
    : htim_(htim), ticksPerRev_(ticksPerRev), direction_(direction), lastCnt_(0), delta_(0), rpm_(0.0f)
{
}

void Encoder::Start()
{
  HAL_TIM_Encoder_Start(htim_, TIM_CHANNEL_ALL);
  lastCnt_ = static_cast<int32_t>(__HAL_TIM_GET_COUNTER(htim_));
}

void Encoder::UpdateRpm(uint32_t dtMs)
{
  const uint32_t arr = __HAL_TIM_GET_AUTORELOAD(htim_);
  const int32_t now = static_cast<int32_t>(__HAL_TIM_GET_COUNTER(htim_));

  delta_ = DeltaWithOverflow(now, lastCnt_, arr);
  lastCnt_ = now;

  const float dt = static_cast<float>(dtMs) / 1000.0f;
  rpm_ = (static_cast<float>(delta_) / ticksPerRev_) * (60.0f / dt) * static_cast<float>(direction_);
}

int32_t Encoder::DeltaWithOverflow(int32_t now, int32_t prev, uint32_t arr)
{
  int32_t delta = now - prev;
  const int32_t modulo = static_cast<int32_t>(arr + 1U);
  const int32_t half = modulo / 2;

  if (delta > half)
  {
    delta -= modulo;
  }

  if (delta < -half)
  {
    delta += modulo;
  }

  return delta;
}
