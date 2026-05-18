#include "omni_kinematics.hpp"

#include <cmath>

namespace
{
constexpr float kPi = 3.14159265358979323846f;
// Electrical order: M1, M2, M3, M4
// Physical placement:
// M1: front-right, M2: front-left, M3: rear-left, M4: rear-right
// Frame: +x right, +y forward, angles CCW from +x.
constexpr float kWheelPhiDeg[4] = {30.0f, 150.0f, 225.0f, 315.0f};
}

OmniKinematics::OmniKinematics(float wheelRadiusM, float robotRadiusM)
    : r_(wheelRadiusM),
      l_(robotRadiusM),
      t_{},
      tPinv_{}
{
  if (r_ <= 0.0f)
  {
    r_ = 0.03f;
  }
  if (l_ <= 0.0f)
  {
    l_ = 0.09f;
  }
  BuildMatrices();
}

void OmniKinematics::RobotToWheels(float vx, float vy, float omega, float outW[4]) const
{
  if (outW == nullptr)
  {
    return;
  }

  for (uint8_t i = 0; i < 4; i++)
  {
    const float u = (t_[i][0] * vx) + (t_[i][1] * vy) + (t_[i][2] * omega);
    outW[i] = u / r_;
  }
}

OmniRobotTwist OmniKinematics::WheelsToRobot(const float wheelW[4]) const
{
  OmniRobotTwist out{0.0f, 0.0f, 0.0f};
  if (wheelW == nullptr)
  {
    return out;
  }

  const float rw[4] = {
      r_ * wheelW[0],
      r_ * wheelW[1],
      r_ * wheelW[2],
      r_ * wheelW[3]};

  out.vx = (tPinv_[0][0] * rw[0]) + (tPinv_[0][1] * rw[1]) + (tPinv_[0][2] * rw[2]) + (tPinv_[0][3] * rw[3]);
  out.vy = (tPinv_[1][0] * rw[0]) + (tPinv_[1][1] * rw[1]) + (tPinv_[1][2] * rw[2]) + (tPinv_[1][3] * rw[3]);
  out.omega = (tPinv_[2][0] * rw[0]) + (tPinv_[2][1] * rw[1]) + (tPinv_[2][2] * rw[2]) + (tPinv_[2][3] * rw[3]);
  return out;
}

bool OmniKinematics::Invert3x3(const float a[3][3], float invOut[3][3])
{
  const float det =
      a[0][0] * (a[1][1] * a[2][2] - a[1][2] * a[2][1]) -
      a[0][1] * (a[1][0] * a[2][2] - a[1][2] * a[2][0]) +
      a[0][2] * (a[1][0] * a[2][1] - a[1][1] * a[2][0]);

  if (std::fabs(det) < 1e-9f)
  {
    return false;
  }

  const float invDet = 1.0f / det;

  invOut[0][0] = (a[1][1] * a[2][2] - a[1][2] * a[2][1]) * invDet;
  invOut[0][1] = (a[0][2] * a[2][1] - a[0][1] * a[2][2]) * invDet;
  invOut[0][2] = (a[0][1] * a[1][2] - a[0][2] * a[1][1]) * invDet;

  invOut[1][0] = (a[1][2] * a[2][0] - a[1][0] * a[2][2]) * invDet;
  invOut[1][1] = (a[0][0] * a[2][2] - a[0][2] * a[2][0]) * invDet;
  invOut[1][2] = (a[0][2] * a[1][0] - a[0][0] * a[1][2]) * invDet;

  invOut[2][0] = (a[1][0] * a[2][1] - a[1][1] * a[2][0]) * invDet;
  invOut[2][1] = (a[0][1] * a[2][0] - a[0][0] * a[2][1]) * invDet;
  invOut[2][2] = (a[0][0] * a[1][1] - a[0][1] * a[1][0]) * invDet;
  return true;
}

void OmniKinematics::BuildMatrices()
{
  for (uint8_t i = 0; i < 4; i++)
  {
    const float phi = kWheelPhiDeg[i] * (kPi / 180.0f);
    const float theta = phi + (kPi * 0.5f); // tangential rolling direction
    t_[i][0] = std::cos(theta);
    t_[i][1] = std::sin(theta);
    t_[i][2] = l_ * std::sin(theta - phi);
  }

  float tt[3][4];
  for (uint8_t r = 0; r < 3; r++)
  {
    for (uint8_t c = 0; c < 4; c++)
    {
      tt[r][c] = t_[c][r];
    }
  }

  float ttT[3][3] = {};
  for (uint8_t i = 0; i < 3; i++)
  {
    for (uint8_t j = 0; j < 3; j++)
    {
      float acc = 0.0f;
      for (uint8_t k = 0; k < 4; k++)
      {
        acc += tt[i][k] * t_[k][j];
      }
      ttT[i][j] = acc;
    }
  }

  float ttTInv[3][3] = {};
  if (!Invert3x3(ttT, ttTInv))
  {
    for (uint8_t i = 0; i < 3; i++)
    {
      for (uint8_t j = 0; j < 4; j++)
      {
        tPinv_[i][j] = 0.0f;
      }
    }
    return;
  }

  // T_pinv = (T^T T)^-1 T^T, size 3x4
  for (uint8_t i = 0; i < 3; i++)
  {
    for (uint8_t j = 0; j < 4; j++)
    {
      float acc = 0.0f;
      for (uint8_t k = 0; k < 3; k++)
      {
        acc += ttTInv[i][k] * tt[k][j];
      }
      tPinv_[i][j] = acc;
    }
  }
}
