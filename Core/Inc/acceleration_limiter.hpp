#pragma once

#include "robot_config.h"

struct LimitedRobotVelocity
{
  float vx;
  float vy;
  float omega;
};

class AccelerationLimiter
{
public:
  explicit AccelerationLimiter(const MotionConfig &config);
  void SetConfig(const MotionConfig &config);

  LimitedRobotVelocity Update(float desiredVx, float desiredVy, float desiredOmega,
                              float dtS, bool braking);
  void Reset();
  LimitedRobotVelocity Applied() const;

private:
  MotionConfig config_;
  float vx_;
  float vy_;
  float omega_;
};
