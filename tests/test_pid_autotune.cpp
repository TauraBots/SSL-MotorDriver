#include "pid_autotune.hpp"

#include <cassert>
#include <cmath>

int main()
{
  PIDAutoTune tune;
  tune.Start(0U);

  float rpm = 0.0f;
  constexpr float dt = 0.001f;
  constexpr float plantGainRpmPerPwm = 0.25f;
  constexpr float plantTimeConstantS = 0.08f;
  uint32_t finishTick = 0U;
  for (uint32_t tick = 0U; tick < AUTOTUNE_TIMEOUT_MS; tick++)
  {
    const int32_t pwm = tune.Update(rpm, tick);
    const float steadyRpm = plantGainRpmPerPwm * static_cast<float>(pwm);
    rpm += ((steadyRpm - rpm) / plantTimeConstantS) * dt;
    if (tune.State() != AUTOTUNE_RUNNING)
    {
      finishTick = tick;
      break;
    }
  }

  assert(tune.State() == AUTOTUNE_FINISHED);
  assert(finishTick >= AUTOTUNE_MINIMUM_RUN_MS);
  assert(tune.Error() == AUTOTUNE_ERROR_NONE);
  assert(tune.Diagnostics().stableWindows >= 3U);
  const PIDAutoTuneResult result = tune.Result();
  assert(std::isfinite(result.tu) && result.tu > 0.02f);
  assert(std::isfinite(result.ku) && result.ku > 0.0f);
  assert(std::fabs(result.kp - (0.45f * result.ku)) < 1.0e-4f);
  assert(std::fabs(result.ki - ((1.2f * result.kp) / result.tu)) < 1.0e-3f);
  assert(result.kd == 0.0f);

  // Encoder RPM is quantized and has deterministic ripple in the real 1 kHz
  // loop. One outlying peak must not reject an otherwise repeatable relay
  // oscillation.
  PIDAutoTune quantizedTune;
  quantizedTune.Start(0U);
  rpm = 0.0f;
  finishTick = 0U;
  for (uint32_t tick = 0U; tick < AUTOTUNE_TIMEOUT_MS; tick++)
  {
    const float ripple = 0.9f * std::sin(static_cast<float>(tick) * 0.037f);
    const float measuredRpm = std::round((rpm + ripple) * 2.0f) * 0.5f;
    const int32_t pwm = quantizedTune.Update(measuredRpm, tick);
    const float steadyRpm = plantGainRpmPerPwm * static_cast<float>(pwm);
    rpm += ((steadyRpm - rpm) / plantTimeConstantS) * dt;
    if (quantizedTune.State() != AUTOTUNE_RUNNING)
    {
      finishTick = tick;
      break;
    }
  }
  assert(quantizedTune.State() == AUTOTUNE_FINISHED);
  assert(finishTick >= AUTOTUNE_MINIMUM_RUN_MS);
  assert((quantizedTune.Diagnostics().stabilityFlags & 0x0fU) == 0x0fU);
  assert(quantizedTune.Diagnostics().stableWindows >= 3U);

  PIDAutoTune stoppedMotor;
  stoppedMotor.Start(0U);
  for (uint32_t tick = 0U; tick <= 1100U; tick++)
  {
    (void)stoppedMotor.Update(0.0f, tick);
  }
  assert(stoppedMotor.State() == AUTOTUNE_FAILED);
  assert(stoppedMotor.Error() == AUTOTUNE_ERROR_ENCODER_NO_MOVEMENT);

  PIDAutoTune aborted;
  aborted.Start(0U);
  aborted.Abort(AUTOTUNE_ERROR_COMMUNICATION_LOST);
  assert(aborted.State() == AUTOTUNE_FAILED);
  assert(aborted.Error() == AUTOTUNE_ERROR_COMMUNICATION_LOST);
  assert(aborted.Update(0.0f, 1U) == 0);
  return 0;
}
