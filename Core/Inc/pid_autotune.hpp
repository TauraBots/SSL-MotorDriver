#pragma once

#include "pid_config.h"

#include <cstdint>

constexpr uint32_t AUTOTUNE_MINIMUM_RUN_MS = 8000U;
constexpr uint32_t AUTOTUNE_TIMEOUT_MS = 30000U;
constexpr int32_t AUTOTUNE_RELAY_PWM = 600;
constexpr int32_t AUTOTUNE_PWM_LIMIT = 800;

enum PIDAutoTuneState : uint8_t
{
  AUTOTUNE_IDLE = 0U,
  AUTOTUNE_RUNNING = 1U,
  AUTOTUNE_FINISHED = 2U,
  AUTOTUNE_FAILED = 3U
};

enum PIDAutoTuneError : uint8_t
{
  AUTOTUNE_ERROR_NONE = 0U,
  AUTOTUNE_ERROR_INVALID_MOTOR = 1U,
  AUTOTUNE_ERROR_BATTERY_LOW = 2U,
  AUTOTUNE_ERROR_ENCODER_NO_MOVEMENT = 3U,
  AUTOTUNE_ERROR_TIMEOUT = 4U,
  AUTOTUNE_ERROR_COMMUNICATION_LOST = 5U,
  AUTOTUNE_ERROR_INVALID_OSCILLATION = 6U,
  AUTOTUNE_ERROR_OVERSPEED = 7U,
  AUTOTUNE_ERROR_FLASH_WRITE = 8U,
  AUTOTUNE_ERROR_ABORTED = 9U
};

struct PIDAutoTuneResult
{
  float tu;
  float ku;
  float kp;
  float ki;
  float kd;
};

struct PIDAutoTuneDiagnostics
{
  uint32_t elapsedMs;
  uint8_t completedPeriods;
  uint8_t usablePeriods;
  uint8_t stabilityFlags;
  uint8_t stableWindows;
  float averageHighRpm;
  float averageLowRpm;
  float periodSpread;
  float highPeakSpread;
  float lowPeakSpread;
};

class PIDAutoTune
{
public:
  PIDAutoTune();

  void Start(uint32_t nowMs);
  int32_t Update(float measuredRpm, uint32_t nowMs);
  void Abort(PIDAutoTuneError error);
  void Reset();

  PIDAutoTuneState State() const { return state_; }
  PIDAutoTuneError Error() const { return error_; }
  PIDAutoTuneResult Result() const { return result_; }
  PIDAutoTuneDiagnostics Diagnostics() const;
  bool IsRunning() const { return state_ == AUTOTUNE_RUNNING; }

private:
  static constexpr uint8_t kSampleCapacity = 8U;
  static constexpr uint8_t kWarmupPeriods = 2U;
  static constexpr uint8_t kRequiredPeriods = 8U;
  static constexpr uint8_t kRequiredStableWindows = 3U;
  static constexpr uint32_t kMinimumSwitchIntervalMs = 30U;
  static constexpr uint32_t kNoMovementTimeoutMs = 1000U;
  static constexpr float kRelayHysteresisRpm = 20.0f;
  static constexpr float kMinimumMovementRpm = 5.0f;
  static constexpr float kMaximumSafeRpm = 450.0f;
  // Relative standard deviation. This is less sensitive to one quantized
  // encoder sample than the previous (maximum - minimum) / average metric.
  static constexpr float kMaximumPeriodSpread = 0.08f;
  static constexpr float kMaximumAmplitudeSpread = 0.08f;
  static constexpr float kMaximumPeakAsymmetry = 0.12f;

  static void PushSample(float value, float samples[kSampleCapacity], uint8_t &count);
  static bool SamplesAreStable(const float samples[kSampleCapacity], uint8_t count,
                               float maximumRelativeSpread);
  static float RelativeSpread(const float samples[kSampleCapacity], uint8_t count);
  bool PeaksAreSymmetric() const;
  bool CalculateResult();
  void Fail(PIDAutoTuneError error);

  PIDAutoTuneState state_;
  PIDAutoTuneError error_;
  PIDAutoTuneResult result_;
  uint32_t elapsedMs_;
  uint32_t startMs_;
  uint32_t lastSwitchMs_;
  uint32_t lastPositiveCrossingMs_;
  bool havePositiveCrossing_;
  bool relayPositive_;
  bool havePendingHighPeak_;
  float phaseMaximumRpm_;
  float phaseMinimumRpm_;
  float pendingHighPeakRpm_;
  float maximumAbsoluteRpm_;
  float highPeaks_[kSampleCapacity];
  float lowPeaks_[kSampleCapacity];
  float periodsS_[kSampleCapacity];
  uint8_t highPeakCount_;
  uint8_t lowPeakCount_;
  uint8_t periodCount_;
  uint8_t completedPeriodCount_;
  uint8_t stableWindowCount_;
};
