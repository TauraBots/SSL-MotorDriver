#pragma once

/*
 * Robot configuration profile.
 *
 * Keep the runtime logical ID out of this file. ROBOT_CONFIG_PROFILE selects
 * compile-time mechanical/motion characteristics and can later be supplied by
 * the build configuration for different robot variants.
 */
#define ROBOT_CONFIG_PROFILE 1

#if ROBOT_CONFIG_PROFILE == 1

/* Motion acceleration limits. */
#define ROBOT_MAX_LINEAR_ACCEL 4.0f   /* m/s^2 */
#define ROBOT_MAX_ANGULAR_ACCEL 10.0f /* rad/s^2 */
#define ROBOT_MAX_BRAKE_ACCEL 7.0f    /* m/s^2 */

/* Motion speed limits. Reserved for the motion-command validation layer. */
#define ROBOT_MAX_LINEAR_SPEED 2.5f  /* m/s */
#define ROBOT_MAX_ANGULAR_SPEED 8.0f /* rad/s */

#else
#error "Unsupported ROBOT_CONFIG_PROFILE"
#endif

#ifdef __cplusplus
struct MotionConfig
{
  float maxLinearAccel;
  float maxAngularAccel;
  float maxBrakeAccel;
  float maxLinearSpeed;
  float maxAngularSpeed;
};

constexpr MotionConfig RobotMotionConfig()
{
  return {
      ROBOT_MAX_LINEAR_ACCEL,
      ROBOT_MAX_ANGULAR_ACCEL,
      ROBOT_MAX_BRAKE_ACCEL,
      ROBOT_MAX_LINEAR_SPEED,
      ROBOT_MAX_ANGULAR_SPEED};
}
#endif
