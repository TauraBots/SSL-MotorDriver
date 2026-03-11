#pragma once

#include <cstdint>

struct OmniRobotTwist
{
  float vx;    // m/s (x to the right)
  float vy;    // m/s (y forward)
  float omega; // rad/s (CCW positive)
};

class OmniKinematics
{
public:
  // Wheel mounting angles (deg): M1=30, M2=150, M3=315, M4=225
  // Rolling direction is tangential: theta_i = phi_i + 90 deg.
  OmniKinematics(float wheelRadiusM = 0.03f, float robotRadiusM = 0.09f);

  // Robot -> wheels (rad/s), order: M1, M2, M3, M4
  void RobotToWheels(float vx, float vy, float omega, float outW[4]) const;

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
