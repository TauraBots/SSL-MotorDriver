#pragma once

// Centralized motion limits. Adjust these values for traction and mass.
#define MAX_LINEAR_ACCEL 3.0f   // m/s^2
#define MAX_ANGULAR_ACCEL 8.0f  // rad/s^2
#define MAX_BRAKE_ACCEL 6.0f    // m/s^2

struct LimitedRobotVelocity
{
  float vx;
  float vy;
  float omega;
};

class AccelerationLimiter
{
public:
  AccelerationLimiter();

  LimitedRobotVelocity Update(float desiredVx, float desiredVy, float desiredOmega,
                              float dtS, bool braking);
  void Reset();
  LimitedRobotVelocity Applied() const;

private:
  float vx_;
  float vy_;
  float omega_;
};
