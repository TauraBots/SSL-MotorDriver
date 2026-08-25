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
  AccelerationLimiter limiter;

  const LimitedRobotVelocity step = limiter.Update(2.0f, 0.0f, 0.0f, kDt, false);
  assert(Near(step.vx, 0.003f));
  assert(Near(step.vy, 0.0f));

  limiter.Reset();
  const LimitedRobotVelocity diagonal = limiter.Update(2.0f, 2.0f, 0.0f, kDt, false);
  assert(Near(std::sqrt((diagonal.vx * diagonal.vx) + (diagonal.vy * diagonal.vy)), 0.003f));
  assert(Near(diagonal.vx, diagonal.vy));

  limiter.Reset();
  for (int i = 0; i < 1000; ++i)
  {
    limiter.Update(2.0f, 0.0f, 0.0f, kDt, false);
  }
  const float beforeBrake = limiter.Applied().vx;
  const LimitedRobotVelocity braking = limiter.Update(0.0f, 0.0f, 0.0f, kDt, true);
  assert(Near(beforeBrake - braking.vx, 0.006f));

  limiter.Reset();
  const LimitedRobotVelocity angular = limiter.Update(0.0f, 0.0f, 5.0f, kDt, false);
  assert(Near(angular.omega, 0.008f));

  limiter.Reset();
  const LimitedRobotVelocity stopped = limiter.Applied();
  assert(Near(stopped.vx, 0.0f));
  assert(Near(stopped.vy, 0.0f));
  assert(Near(stopped.omega, 0.0f));
  return 0;
}
