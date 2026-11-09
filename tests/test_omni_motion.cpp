#include "acceleration_limiter.hpp"
#include "omni_kinematics.hpp"
#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstdio>
#include <limits>

namespace {
constexpr float dt = 0.001f;
constexpr float pi = 3.14159265358979323846f;
void near(float a, float b, float eps = 2.e-4f) { assert(std::fabs(a-b) < eps); }
float norm(float x, float y) { return std::sqrt(x*x+y*y); }

// The simulated actual yaw is independent of commanded yaw. A 15 ms delayed
// measurement bounds the nominal 10 ms window + scheduling age (no slip model).
float globalRun(bool compensate, int delay, bool variableYaw) {
  AccelerationLimiter limiter(RobotMotionConfig());
  OmniKinematics kin;
  float theta = 0, maxError = 0;
  for (int i=0; i<12000; ++i) {
    const float t = i*dt;
    const float actualOmega = variableYaw ? 5.0f*std::sin(t) : 5.0f;
    const float delayedOmega = variableYaw ? 5.0f*std::sin(std::max(0.0f,t-delay*dt)) : 5.0f;
    theta += actualOmega*dt;
    const float c=std::cos(theta), s=std::sin(theta);
    // Speed 1 m/s, so frame variation 5 m/s^2 exceeds the 4 m/s^2 limit.
    const auto v=limiter.Update(c,-s,8.0f,dt,false,compensate ? delayedOmega*dt : 0);
    float wheels[4], raw[4]; kin.RobotToWheels(v.vx,v.vy,v.omega,wheels,raw);
    // This D/E run must be free of wheel saturation.
    for (int j=0;j<4;++j) near(wheels[j],raw[j]);
    const auto reconstructed=kin.WheelsToRobot(wheels);
    const float error=norm(c*reconstructed.vx-s*reconstructed.vy-1,
                           s*reconstructed.vx+c*reconstructed.vy);
    if(i>3000) maxError=std::max(maxError,error);
  }
  return maxError;
}
}

int main() {
  OmniKinematics kin;
  float w[4], raw[4];
  // A/H: full circle and reverse, round trip below wheel saturation.
  for(int i=0;i<360;++i) {
    const float x=0.6f*std::cos(i*pi/180), y=0.6f*std::sin(i*pi/180);
    kin.RobotToWheels(x,y,0,w);
    const auto v=kin.WheelsToRobot(w); near(v.vx,x); near(v.vy,y); near(v.omega,0);
  }
  // B: independent angular expectation from the measured geometry.
  kin.RobotToWheels(0,0,5,w,raw);
  for(float x:raw) near(x*60/(2*pi),116.019f,0.002f);
  std::printf("B pure yaw: %.6f RPM per wheel\n",raw[0]*60/(2*pi));
  // C: independently constructed tangent projections, preserve M1..M4.
  const float angles[]={30,150,225,315};
  kin.RobotToWheels(0.4f,-0.3f,2,w,raw);
  for(int i=0;i<4;++i) {
    const float phi=angles[i]*pi/180;
    const float expected=(-std::sin(phi)*0.4f+std::cos(phi)*(-0.3f)+0.07715f*2)/0.03175f;
    near(raw[i],expected); near(w[i],expected);
    std::printf("C M%d: %.6f RPM\n",i+1,w[i]*60/(2*pi));
  }
  const float fixed=globalRun(true,0,false), old=globalRun(false,0,false);
  const float delayed=globalRun(true,15,true);
  assert(fixed<0.0003f); assert(old>0.1f); assert(delayed<0.003f);
  std::printf("D/E max global error: corrected %.8f, old %.6f, delayed variable yaw %.8f m/s\n",fixed,old,delayed);
  // E: arbitrary real global changes remain limited after frame transport.
  AccelerationLimiter limiter(RobotMotionConfig());
  float peakAccel=0;
  for(int i=0;i<4000;++i) {
    const auto previous=limiter.Applied();
    const float angle=(i%2 ? -5.0f:5.0f)*dt;
    const auto v=limiter.Update(i<2000?2.f:-2.f,1,5,dt,false,angle);
    const float px=std::cos(angle)*previous.vx+std::sin(angle)*previous.vy;
    const float py=-std::sin(angle)*previous.vx+std::cos(angle)*previous.vy;
    const float a=norm(v.vx-px,v.vy-py)/dt;
    peakAccel=std::max(peakAccel,a); assert(a<4.0003f);
    assert(std::fabs(v.omega-previous.omega)<0.01001f);
  }
  std::printf("E peak global acceleration: %.6f m/s^2\n",peakAccel);
  // F: proportional saturation scales the complete twist by one factor.
  kin.RobotToWheels(2.5f,-1.7f,8,w,raw);
  float maxRaw=0,maxW=0;
  for(int i=0;i<4;++i){maxRaw=std::max(maxRaw,std::fabs(raw[i]));maxW=std::max(maxW,std::fabs(w[i]));}
  const float scale=maxW/maxRaw; near(maxW*60/(2*pi),500,0.001f);
  const auto v=kin.WheelsToRobot(w);
  near(v.vx,2.5f*scale);near(v.vy,-1.7f*scale);near(v.omega,8*scale);
  std::printf("F scale %.6f, reconstructed twist %.6f %.6f %.6f\n",scale,v.vx,v.vy,v.omega);
  const float nan=std::numeric_limits<float>::quiet_NaN();
  const float huge=std::numeric_limits<float>::max();
  kin.RobotToWheels(huge,huge,huge,w,raw);
  for(float x:w) assert(std::isfinite(x));
  for(float x:raw) assert(std::isfinite(x));
  kin.RobotToWheels(nan,0,0,w,raw);for(float x:w) near(x,0);
  kin.RobotToWheels(0,0,std::numeric_limits<float>::infinity(),w,raw);
  for(float x:w) near(x,0);
  // G: braking, inversion, reset/restart, invalid inputs, speed bounds.
  for(int i=0;i<1000;++i) limiter.Update(2,0,0,dt,false);
  auto before=limiter.Applied(); auto after=limiter.Update(0,0,0,dt,true);
  near(before.vx-after.vx,7*dt);
  for(int i=0;i<1000;++i) limiter.Update(0,0,0,dt,true);
  near(limiter.Applied().vx,0);near(limiter.Applied().vy,0);
  limiter.Reset(); after=limiter.Update(-2,0,0,dt,false);near(after.vx,-4*dt);
  limiter.Update(nan,0,0,dt,false);near(limiter.Applied().vx,0);
  limiter.Update(1,1,1,nan,false);near(limiter.Applied().omega,0);
  for(int i=0;i<2000;++i) after=limiter.Update(huge,huge,huge,dt,false);
  near(norm(after.vx,after.vy),2.5f);near(after.omega,8);
  const auto saved=after;
  auto badConfig=RobotMotionConfig();badConfig.maxLinearAccel=nan;limiter.SetConfig(badConfig);
  near(limiter.Applied().vx,saved.vx);
  after=limiter.Update(0,0,0,0,false);near(after.vx,saved.vx);
  std::puts("A-H motion checks passed (G watchdog/integration in test_motion_pipeline)");
}
