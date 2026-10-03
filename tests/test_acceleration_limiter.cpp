#include "acceleration_limiter.hpp"

#include <cassert>
#include <cmath>

namespace
{
constexpr float kDt = 0.001f;

bool Near(float a, float b, float tolerance = 1.0e-5f)
{
  return std::fabs(a - b) <= tolerance;
}
}

int main()
{
  const MotionConfig config = RobotMotionConfig();
  AccelerationLimiter limiter(config);

  const LimitedRobotVelocity step = limiter.Update(2.0f, 0.0f, 0.0f, kDt, false);
  assert(Near(step.vx, config.maxLinearAccel * kDt));
  assert(Near(step.vy, 0.0f));

  limiter.Reset();
  const LimitedRobotVelocity diagonal = limiter.Update(2.0f, 2.0f, 0.0f, kDt, false);
  assert(Near(std::sqrt((diagonal.vx * diagonal.vx) + (diagonal.vy * diagonal.vy)),
              config.maxLinearAccel * kDt));
  assert(Near(diagonal.vx, diagonal.vy));

  limiter.Reset();
  for (int i = 0; i < 1000; ++i)
  {
    limiter.Update(2.0f, 0.0f, 0.0f, kDt, false);
  }
  const float beforeNormalStop = limiter.Applied().vx;
  const LimitedRobotVelocity normalStop = limiter.Update(0.0f, 0.0f, 0.0f, kDt, false);
  assert(Near(beforeNormalStop - normalStop.vx, config.maxLinearAccel * kDt));

  limiter.Reset();
  for (int i = 0; i < 1000; ++i)
  {
    limiter.Update(2.0f, 0.0f, 0.0f, kDt, false);
  }
  const float beforeBrake = limiter.Applied().vx;
  const LimitedRobotVelocity braking = limiter.Update(0.0f, 0.0f, 0.0f, kDt, true);
  assert(Near(beforeBrake - braking.vx, config.maxBrakeAccel * kDt));

  limiter.Reset();
  const LimitedRobotVelocity angular = limiter.Update(0.0f, 0.0f, 5.0f, kDt, false);
  assert(Near(angular.omega, config.maxAngularAccel * kDt));

  const MotionConfig gentleConfig = {1.0f, 2.0f, 3.0f, 2.5f, 8.0f};
  AccelerationLimiter gentleLimiter(gentleConfig);
  const LimitedRobotVelocity configured =
      gentleLimiter.Update(2.0f, 0.0f, 5.0f, kDt, false);
  assert(Near(configured.vx, 0.001f));
  assert(Near(configured.omega, 0.002f));

  limiter.Reset();
  const LimitedRobotVelocity stopped = limiter.Applied();
  assert(Near(stopped.vx, 0.0f));
  assert(Near(stopped.vy, 0.0f));
  assert(Near(stopped.omega, 0.0f));
  return 0;
}
