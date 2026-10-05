#include "pid_autotune.hpp"

#include <algorithm>
#include <cmath>

namespace
{
constexpr float kPi = 3.14159265358979323846f;

float Average(const float *values, uint8_t count)
{
  if ((values == nullptr) || (count == 0U))
  {
    return 0.0f;
  }

  float sum = 0.0f;
  for (uint8_t i = 0U; i < count; i++)
  {
    sum += values[i];
  }
  return sum / static_cast<float>(count);
}
}

PIDAutoTune::PIDAutoTune()
{
  Reset();
}

void PIDAutoTune::Reset()
{
  state_ = AUTOTUNE_IDLE;
  error_ = AUTOTUNE_ERROR_NONE;
  result_ = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
  elapsedMs_ = 0U;
  startMs_ = 0U;
  lastSwitchMs_ = 0U;
  lastPositiveCrossingMs_ = 0U;
  havePositiveCrossing_ = false;
  relayPositive_ = true;
  havePendingHighPeak_ = false;
  phaseMaximumRpm_ = 0.0f;
  phaseMinimumRpm_ = 0.0f;
  pendingHighPeakRpm_ = 0.0f;
  maximumAbsoluteRpm_ = 0.0f;
  std::fill_n(highPeaks_, kSampleCapacity, 0.0f);
  std::fill_n(lowPeaks_, kSampleCapacity, 0.0f);
  std::fill_n(periodsS_, kSampleCapacity, 0.0f);
  highPeakCount_ = 0U;
  lowPeakCount_ = 0U;
  periodCount_ = 0U;
  completedPeriodCount_ = 0U;
  stableWindowCount_ = 0U;
}

void PIDAutoTune::Start(uint32_t nowMs)
{
  Reset();
  state_ = AUTOTUNE_RUNNING;
  startMs_ = nowMs;
  lastSwitchMs_ = nowMs;
}

void PIDAutoTune::PushSample(float value, float samples[kSampleCapacity], uint8_t &count)
{
  if (count < kSampleCapacity)
  {
    samples[count++] = value;
    return;
  }

  for (uint8_t i = 1U; i < kSampleCapacity; i++)
  {
    samples[i - 1U] = samples[i];
  }
  samples[kSampleCapacity - 1U] = value;
}

bool PIDAutoTune::SamplesAreStable(const float samples[kSampleCapacity], uint8_t count,
                                   float maximumRelativeSpread)
{
  return (count >= kRequiredPeriods) &&
         (RelativeSpread(samples, count) <= maximumRelativeSpread);
}

float PIDAutoTune::RelativeSpread(const float samples[kSampleCapacity], uint8_t count)
{
  if (count < 2U)
  {
    return 0.0f;
  }
  const float average = Average(samples, count);
  if (!std::isfinite(average) || (std::fabs(average) <= 1.0e-6f))
  {
    return 1.0e9f;
  }

  float squaredDeviationSum = 0.0f;
  for (uint8_t i = 0U; i < count; i++)
  {
    const float deviation = samples[i] - average;
    squaredDeviationSum += deviation * deviation;
  }
  const float standardDeviation =
      std::sqrt(squaredDeviationSum / static_cast<float>(count - 1U));
  return standardDeviation / std::fabs(average);
}

bool PIDAutoTune::PeaksAreSymmetric() const
{
  if ((highPeakCount_ < kRequiredPeriods) ||
      (lowPeakCount_ < kRequiredPeriods))
  {
    return false;
  }

  const float highMagnitude = std::fabs(Average(highPeaks_, highPeakCount_));
  const float lowMagnitude = std::fabs(Average(lowPeaks_, lowPeakCount_));
  const float meanMagnitude = 0.5f * (highMagnitude + lowMagnitude);
  if (!std::isfinite(meanMagnitude) || (meanMagnitude <= 1.0e-6f))
  {
    return false;
  }
  return (std::fabs(highMagnitude - lowMagnitude) / meanMagnitude) <=
         kMaximumPeakAsymmetry;
}

PIDAutoTuneDiagnostics PIDAutoTune::Diagnostics() const
{
  uint8_t stabilityFlags = 0U;
  if (SamplesAreStable(periodsS_, periodCount_, kMaximumPeriodSpread))
  {
    stabilityFlags |= 0x01U;
  }
  if (SamplesAreStable(highPeaks_, highPeakCount_, kMaximumAmplitudeSpread))
  {
    stabilityFlags |= 0x02U;
  }
  if (SamplesAreStable(lowPeaks_, lowPeakCount_, kMaximumAmplitudeSpread))
  {
    stabilityFlags |= 0x04U;
  }
  if (PeaksAreSymmetric())
  {
    stabilityFlags |= 0x08U;
  }
  return {
      elapsedMs_, completedPeriodCount_, periodCount_, stabilityFlags,
      stableWindowCount_,
      Average(highPeaks_, highPeakCount_), Average(lowPeaks_, lowPeakCount_),
      RelativeSpread(periodsS_, periodCount_),
      RelativeSpread(highPeaks_, highPeakCount_),
      RelativeSpread(lowPeaks_, lowPeakCount_)};
}

int32_t PIDAutoTune::Update(float measuredRpm, uint32_t nowMs)
{
  if (state_ != AUTOTUNE_RUNNING)
  {
    return 0;
  }

  if (!std::isfinite(measuredRpm))
  {
    Fail(AUTOTUNE_ERROR_INVALID_OSCILLATION);
    return 0;
  }

  const uint32_t elapsedMs = nowMs - startMs_;
  elapsedMs_ = elapsedMs;
  const float absoluteRpm = std::fabs(measuredRpm);
  maximumAbsoluteRpm_ = std::max(maximumAbsoluteRpm_, absoluteRpm);
  phaseMaximumRpm_ = std::max(phaseMaximumRpm_, measuredRpm);
  phaseMinimumRpm_ = std::min(phaseMinimumRpm_, measuredRpm);

  if (absoluteRpm > kMaximumSafeRpm)
  {
    Fail(AUTOTUNE_ERROR_OVERSPEED);
    return 0;
  }
  if ((elapsedMs >= kNoMovementTimeoutMs) &&
      (maximumAbsoluteRpm_ < kMinimumMovementRpm))
  {
    Fail(AUTOTUNE_ERROR_ENCODER_NO_MOVEMENT);
    return 0;
  }
  if (elapsedMs >= AUTOTUNE_TIMEOUT_MS)
  {
    const bool oscillated = (periodCount_ >= kRequiredPeriods) &&
                            (highPeakCount_ >= kRequiredPeriods) &&
                            (lowPeakCount_ >= kRequiredPeriods);
    Fail(oscillated ? AUTOTUNE_ERROR_INVALID_OSCILLATION : AUTOTUNE_ERROR_TIMEOUT);
    return 0;
  }

  const uint32_t sinceSwitchMs = nowMs - lastSwitchMs_;
  bool sampleWindowAdvanced = false;
  if (relayPositive_ && (measuredRpm >= kRelayHysteresisRpm) &&
      (sinceSwitchMs >= kMinimumSwitchIntervalMs))
  {
    if (havePositiveCrossing_)
    {
      if (completedPeriodCount_ < 0xFFU)
      {
        completedPeriodCount_++;
      }
      if ((completedPeriodCount_ > kWarmupPeriods) && havePendingHighPeak_ &&
          (phaseMinimumRpm_ <= -kRelayHysteresisRpm))
      {
        PushSample(static_cast<float>(nowMs - lastPositiveCrossingMs_) * 0.001f,
                   periodsS_, periodCount_);
        PushSample(pendingHighPeakRpm_, highPeaks_, highPeakCount_);
        PushSample(phaseMinimumRpm_, lowPeaks_, lowPeakCount_);
        sampleWindowAdvanced = true;
      }
    }
    havePendingHighPeak_ = false;
    lastPositiveCrossingMs_ = nowMs;
    havePositiveCrossing_ = true;
    relayPositive_ = false;
    lastSwitchMs_ = nowMs;
    phaseMaximumRpm_ = measuredRpm;
    phaseMinimumRpm_ = measuredRpm;
  }
  else if (!relayPositive_ && (measuredRpm <= -kRelayHysteresisRpm) &&
           (sinceSwitchMs >= kMinimumSwitchIntervalMs))
  {
    if (phaseMaximumRpm_ >= kRelayHysteresisRpm)
    {
      pendingHighPeakRpm_ = phaseMaximumRpm_;
      havePendingHighPeak_ = true;
    }
    relayPositive_ = true;
    lastSwitchMs_ = nowMs;
    phaseMaximumRpm_ = measuredRpm;
    phaseMinimumRpm_ = measuredRpm;
  }

  if (sampleWindowAdvanced && (elapsedMs >= AUTOTUNE_MINIMUM_RUN_MS))
  {
    const bool stable =
        SamplesAreStable(periodsS_, periodCount_, kMaximumPeriodSpread) &&
        SamplesAreStable(highPeaks_, highPeakCount_, kMaximumAmplitudeSpread) &&
        SamplesAreStable(lowPeaks_, lowPeakCount_, kMaximumAmplitudeSpread) &&
        PeaksAreSymmetric();
    if (stable)
    {
      if (stableWindowCount_ < kRequiredStableWindows)
      {
        stableWindowCount_++;
      }
    }
    else
    {
      stableWindowCount_ = 0U;
    }
  }

  const bool enoughStableSamples =
      (periodCount_ >= kRequiredPeriods) &&
      (highPeakCount_ >= kRequiredPeriods) &&
      (lowPeakCount_ >= kRequiredPeriods) &&
      SamplesAreStable(periodsS_, periodCount_, kMaximumPeriodSpread) &&
      SamplesAreStable(highPeaks_, highPeakCount_, kMaximumAmplitudeSpread) &&
      SamplesAreStable(lowPeaks_, lowPeakCount_, kMaximumAmplitudeSpread) &&
      PeaksAreSymmetric() &&
      (stableWindowCount_ >= kRequiredStableWindows);
  if ((elapsedMs >= AUTOTUNE_MINIMUM_RUN_MS) && enoughStableSamples)
  {
    if (!CalculateResult())
    {
      Fail(AUTOTUNE_ERROR_INVALID_OSCILLATION);
      return 0;
    }
    state_ = AUTOTUNE_FINISHED;
    return 0;
  }

  (void)CalculateResult();
  const int32_t requested = relayPositive_ ? AUTOTUNE_RELAY_PWM : -AUTOTUNE_RELAY_PWM;
  return std::max(-AUTOTUNE_PWM_LIMIT, std::min(AUTOTUNE_PWM_LIMIT, requested));
}

bool PIDAutoTune::CalculateResult()
{
  if ((periodCount_ < 2U) || (highPeakCount_ < 2U) || (lowPeakCount_ < 2U))
  {
    return false;
  }

  const float tu = Average(periodsS_, periodCount_);
  const float high = Average(highPeaks_, highPeakCount_);
  const float low = Average(lowPeaks_, lowPeakCount_);
  const float processAmplitude = (high - low) * 0.5f;
  if (!std::isfinite(tu) || !std::isfinite(processAmplitude) ||
      (tu < 0.02f) || (tu > 5.0f) || (processAmplitude < 5.0f))
  {
    return false;
  }

  const float ku = (4.0f * static_cast<float>(AUTOTUNE_RELAY_PWM)) /
                   (kPi * processAmplitude);
  const float kp = 0.45f * ku;
  const float ki = (1.2f * kp) / tu;
  if (!std::isfinite(ku) || !std::isfinite(kp) || !std::isfinite(ki) ||
      (kp <= 0.0f) || (kp > 100.0f) || (ki <= 0.0f) || (ki > 2000.0f))
  {
    return false;
  }

  result_ = {tu, ku, kp, ki, 0.0f};
  return true;
}

void PIDAutoTune::Abort(PIDAutoTuneError error)
{
  if (error == AUTOTUNE_ERROR_NONE)
  {
    error = AUTOTUNE_ERROR_ABORTED;
  }
  Fail(error);
}

void PIDAutoTune::Fail(PIDAutoTuneError error)
{
  state_ = AUTOTUNE_FAILED;
  error_ = error;
}
