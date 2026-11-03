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

  // Measured CCW rotation since previous Update; zero disables frame compensation.
  // Never substitute commanded omega for measured rotation.
  LimitedRobotVelocity Update(float desiredVx, float desiredVy, float desiredOmega,
                              float dtS, bool braking, float deltaTheta = 0.0f);
  void Reset();
  LimitedRobotVelocity Applied() const;

private:
  MotionConfig config_;
  float vx_;
  float vy_;
  float omega_;
};
