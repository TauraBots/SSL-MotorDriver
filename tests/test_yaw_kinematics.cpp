#include "omni_kinematics.hpp"

#include <cassert>
#include <cmath>
#include <cstdio>
#include <initializer_list>

namespace
{
constexpr float kRadSToRpm = 9.5492966f;
constexpr float kNoteWheelRadiusM = 0.03175f;
constexpr float kNoteRobotRadiusM = 0.07715f;
constexpr float kFirmwareWheelRadiusM = 0.03f;
constexpr float kFirmwareRobotRadiusM = 0.09f;

bool Near(float a, float b, float tolerance = 0.05f)
{
  return std::fabs(a - b) <= tolerance;
}
}

int main()
{
  OmniKinematics kinematics(kFirmwareWheelRadiusM, kFirmwareRobotRadiusM);
  std::printf("omega  note_rpm  firmware_raw_rpm  firmware_limited_rpm\n");
  for (const float omega : {1.0f, 5.0f, 10.0f, 20.0f})
  {
    float limited[4] = {};
    float raw[4] = {};
    kinematics.RobotToWheels(0.0f, 0.0f, omega, limited, raw);
    const float noteRpm = omega * kNoteRobotRadiusM /
                          kNoteWheelRadiusM * kRadSToRpm;
    const float firmwareRawRpm = omega * kFirmwareRobotRadiusM /
                                 kFirmwareWheelRadiusM * kRadSToRpm;
    const float firmwareLimitedRpm = std::fmin(firmwareRawRpm, 500.0f);
    std::printf("%5.1f  %8.2f  %16.2f  %20.2f\n",
                omega, noteRpm, raw[0] * kRadSToRpm,
                limited[0] * kRadSToRpm);
    for (unsigned i = 0; i < 4; i++)
    {
      assert(Near(std::fabs(raw[i] * kRadSToRpm), firmwareRawRpm));
      assert(Near(std::fabs(limited[i] * kRadSToRpm), firmwareLimitedRpm));
    }
  }
  return 0;
}
