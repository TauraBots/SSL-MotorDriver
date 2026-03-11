#include "motor.hpp"

namespace
{
constexpr uint16_t kArr30kHz = 2399U;
}

Motor::Motor(TIM_HandleTypeDef *htim, uint32_t channelA, uint32_t channelB, bool complementary)
    : htim_(htim), channelA_(channelA), channelB_(channelB), complementary_(complementary)
{
}

void Motor::Start() const
{
  HAL_TIM_PWM_Start(htim_, channelA_);
  HAL_TIM_PWM_Start(htim_, channelB_);

  if (complementary_)
  {
    HAL_TIMEx_PWMN_Start(htim_, channelA_);
    HAL_TIMEx_PWMN_Start(htim_, channelB_);
    __HAL_TIM_MOE_ENABLE(htim_);
  }

  __HAL_TIM_SET_COMPARE(htim_, channelA_, 0);
  __HAL_TIM_SET_COMPARE(htim_, channelB_, 0);
}

void Motor::ApplySigned(int32_t cmd, bool brakeMode) const
{
  if (cmd == 0)
  {
    Stop(brakeMode);
    return;
  }

  const uint16_t duty = ClampDuty(cmd > 0 ? cmd : -cmd);

  if (cmd > 0)
  {
    __HAL_TIM_SET_COMPARE(htim_, channelB_, 0);
    __HAL_TIM_SET_COMPARE(htim_, channelA_, duty);
  }
  else
  {
    __HAL_TIM_SET_COMPARE(htim_, channelA_, 0);
    __HAL_TIM_SET_COMPARE(htim_, channelB_, duty);
  }
}

uint16_t Motor::ClampDuty(int32_t duty)
{
  if (duty < 0)
  {
    return 0;
  }

  if (duty > static_cast<int32_t>(kArr30kHz))
  {
    return kArr30kHz;
  }

  return static_cast<uint16_t>(duty);
}

void Motor::Stop(bool brakeMode) const
{
  if (brakeMode)
  {
    __HAL_TIM_SET_COMPARE(htim_, channelA_, kArr30kHz);
    __HAL_TIM_SET_COMPARE(htim_, channelB_, kArr30kHz);
    return;
  }

  __HAL_TIM_SET_COMPARE(htim_, channelA_, 0);
  __HAL_TIM_SET_COMPARE(htim_, channelB_, 0);
}
