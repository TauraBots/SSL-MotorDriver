#include "acceleration_limiter.hpp"

#include <algorithm>
#include <cmath>

AccelerationLimiter::AccelerationLimiter(const MotionConfig &config)
    : config_(RobotMotionConfig()), vx_(0.0f), vy_(0.0f), omega_(0.0f)
{
  SetConfig(config);
}

void AccelerationLimiter::SetConfig(const MotionConfig &config)
{
  if (!std::isfinite(config.maxLinearAccel) || config.maxLinearAccel <= 0.0f ||
      !std::isfinite(config.maxAngularAccel) || config.maxAngularAccel <= 0.0f ||
      !std::isfinite(config.maxBrakeAccel) || config.maxBrakeAccel <= 0.0f ||
      !std::isfinite(config.maxLinearSpeed) || config.maxLinearSpeed <= 0.0f ||
      !std::isfinite(config.maxAngularSpeed) || config.maxAngularSpeed <= 0.0f)
    return; // Keep the previous valid configuration and state.
  config_ = config;
  Reset();
}

LimitedRobotVelocity AccelerationLimiter::Update(float desiredVx, float desiredVy,
                                                  float desiredOmega, float dtS,
                                                  bool braking, float deltaTheta)
{
  if (!std::isfinite(desiredVx) || !std::isfinite(desiredVy) ||
      !std::isfinite(desiredOmega) || !std::isfinite(deltaTheta) ||
      !std::isfinite(dtS))
  {
    Reset();
    return Applied();
  }
  if (dtS <= 0.0f)
  {
    return Applied();
  }

  // Normalize first to avoid overflow for extreme but finite commands.
  const float largest = std::max(std::fabs(desiredVx), std::fabs(desiredVy));
  if (largest > 0.0f)
  {
    const float nx = desiredVx / largest;
    const float ny = desiredVy / largest;
    const float norm = std::sqrt(nx * nx + ny * ny);
    if (largest > config_.maxLinearSpeed / norm)
    {
      desiredVx = nx * (config_.maxLinearSpeed / norm);
      desiredVy = ny * (config_.maxLinearSpeed / norm);
    }
  }
  desiredOmega = std::max(-config_.maxAngularSpeed,
                          std::min(config_.maxAngularSpeed, desiredOmega));
  // R(-deltaTheta): +x right, +y forward, yaw positive CCW.
  const float c = std::cos(deltaTheta);
  const float s = std::sin(deltaTheta);
  const float previousX = vx_;
  vx_ = c * previousX + s * vy_;
  vy_ = -s * previousX + c * vy_;
  const float deltaVx = desiredVx - vx_;
  const float deltaVy = desiredVy - vy_;
  const float deltaMagnitude = std::sqrt((deltaVx * deltaVx) + (deltaVy * deltaVy));
  const float maxLinearDelta =
      (braking ? config_.maxBrakeAccel : config_.maxLinearAccel) * dtS;
  if ((deltaMagnitude > maxLinearDelta) && (deltaMagnitude > 1.0e-9f))
  {
    const float scale = maxLinearDelta / deltaMagnitude;
    vx_ += deltaVx * scale;
    vy_ += deltaVy * scale;
  }
  else
  {
    vx_ = desiredVx;
    vy_ = desiredVy;
  }

  const float maxAngularDelta = config_.maxAngularAccel * dtS;
  const float deltaOmega = desiredOmega - omega_;
  omega_ += std::max(-maxAngularDelta, std::min(maxAngularDelta, deltaOmega));
  return Applied();
}

void AccelerationLimiter::Reset()
{
  vx_ = 0.0f;
  vy_ = 0.0f;
  omega_ = 0.0f;
}

LimitedRobotVelocity AccelerationLimiter::Applied() const
{
  return {vx_, vy_, omega_};
}
