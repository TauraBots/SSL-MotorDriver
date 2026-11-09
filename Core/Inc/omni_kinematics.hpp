#pragma once

#include <cstdint>
#include "robot_config.h"

struct OmniRobotTwist
{
  float vx;    // m/s (x to the right)
  float vy;    // m/s (y forward)
  float omega; // rad/s (CCW positive)
};

class OmniKinematics
{
public:
  // Wheel mounting angles (deg): M1=30, M2=150, M3=225, M4=315
  // Rolling direction is tangential: theta_i = phi_i + 90 deg.
  OmniKinematics(float wheelRadiusM = ROBOT_WHEEL_RADIUS_M,
                 float robotRadiusM = ROBOT_CENTER_WHEEL_DISTANCE_M);

  // Robot -> wheels in rad/s. rawW, when provided, captures the values before
  // the common 500 RPM wheel saturation. omega is always expressed in rad/s.
  void RobotToWheels(float vx, float vy, float omega, float outW[4],
                     float rawW[4] = nullptr) const;

  // Wheels -> robot
  OmniRobotTwist WheelsToRobot(const float wheelW[4]) const;

private:
  float r_;
  float l_;
  float t_[4][3];
  float tPinv_[3][4];

  static bool Invert3x3(const float a[3][3], float invOut[3][3]);
  void BuildMatrices();
};
