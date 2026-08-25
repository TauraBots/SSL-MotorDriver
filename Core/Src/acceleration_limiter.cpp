#include "acceleration_limiter.hpp"

#include <algorithm>
#include <cmath>

AccelerationLimiter::AccelerationLimiter()
    : vx_(0.0f), vy_(0.0f), omega_(0.0f)
{
}

LimitedRobotVelocity AccelerationLimiter::Update(float desiredVx, float desiredVy,
                                                  float desiredOmega, float dtS,
                                                  bool braking)
{
  if (dtS <= 0.0f)
  {
    return Applied();
  }

  const float deltaVx = desiredVx - vx_;
  const float deltaVy = desiredVy - vy_;
  const float deltaMagnitude = std::sqrt((deltaVx * deltaVx) + (deltaVy * deltaVy));
  const float maxLinearDelta = (braking ? MAX_BRAKE_ACCEL : MAX_LINEAR_ACCEL) * dtS;
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

  const float maxAngularDelta = MAX_ANGULAR_ACCEL * dtS;
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
