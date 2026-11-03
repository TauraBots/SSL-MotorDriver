#include <cstdint>
#include <cmath>
#include <cassert>
#include <cstdio>
#include <limits>
#include <initializer_list>
// Inspect PID state to verify anti-windup without replacing the real controller.
#define private public
#include "app.hpp"
#undef private
#include "app_c_api.h"
#include "omni_kinematics.hpp"

TIM_TypeDef host_tim1{},host_tim8{};
uint32_t host_tick=0,host_adc=3800;
extern "C" uint8_t KickerBoard_RequestSafeStop() { return 1; }
extern "C" uint8_t KickerBoard_RequestAutoKick(uint8_t) { return 1; }
extern "C" void KickerBoard_CancelPendingKick() {}
void near(float a,float b,float e=0.0002f){assert(std::fabs(a-b)<e);}

int main() {
  TIM_TypeDef enc[4]{};
  TIM_HandleTypeDef et[4];
  for(int i=0;i<4;++i){enc[i].ARR=65535;et[i].Instance=&enc[i];}
  TIM_HandleTypeDef pwm1{&host_tim1},pwm8{&host_tim8};
  ADC_HandleTypeDef adc{};GPIO_TypeDef gpio{};
  // Real Encoder wrap handling, output shaft scaling and M3 sign.
  Encoder encoder(&et[0],11*4*18.8f,-1);
  enc[0].CNT=65530;encoder.Start();enc[0].CNT=4;encoder.UpdateRpm(10,10);
  near(encoder.Rpm(),-10*6000/(11*4*18.8f));
  enc[0].CNT=65530;encoder.UpdateRpm(10,10);
  near(encoder.Rpm(),10*6000/(11*4*18.8f));
  AppC_Init(&adc,&pwm1,&pwm8,&et[0],&et[1],&et[2],&et[3],&gpio,0);
  host_tick=100;AppC_Tick(); // Battery becomes valid.
  AppC_SetRobotVelocity(1,0.5f,0.2f,3,0,0);AppC_FastTick1kHz();
  assert(setpoint_m1!=0);assert(dbg_motion_rotation_source==1);
  // Frozen sequence must not renew the watchdog; test the actual 1 kHz entry.
  host_tick=249;AppC_SetRobotVelocity(1,1,0,0,0,0);AppC_FastTick1kHz();
  AppC_Telemetry telem{};AppC_GetTelemetry(&telem);assert(telem.communication_ok);
  host_tick=250;AppC_FastTick1kHz();AppC_GetTelemetry(&telem);
  assert(!telem.communication_ok);near(setpoint_m1,0);assert(cmd_m1==0);assert(stop_mode_brake);
  // New transmitter epoch after loss, no old limiter state.
  AppC_SetRobotVelocity(0,-1,0,0,0,0);AppC_FastTick1kHz();
  near(dbg_motion_limited_xy[0],-0.004f);near(dbg_motion_limited_xy[1],0);
  // All four actual encoder signs represent positive yaw, independently of
  // the requested omega (zero here). The limiter must use measured rotation.
  for(int i=0;i<20;++i) {
    ++host_tick;
    if(i%10==0) for(int j=0;j<4;++j)
      enc[j].CNT=(enc[j].CNT+(j==2?65520:16))%65536;
    AppC_SetRobotVelocity(i+2,0,0,0,0,0);AppC_FastTick1kHz();
  }
  const float expectedYaw=(16*6000/(11*4*18.8f))*0.1047197551f*0.03175f/0.07715f;
  near(dbg_motion_measured_omega,expectedYaw);
  near(dbg_motion_delta_theta,expectedYaw*0.001f);
  // Saturation trace and references agree; keep communication alive.
  for(int i=0;i<1000;++i){++host_tick;for(int j=0;j<4;++j)enc[j].CNT=(enc[j].CNT+(j==3?2:65534))%65536;AppC_SetRobotVelocity(i+22,2.5f,-1,8,0,0);AppC_FastTick1kHz();}
  assert(dbg_motion_saturation_scale<1);
  near(setpoint_m1,dbg_yaw_wheel_limited_rpm[0]);
  // Invalid packets stop and clear state rather than poisoning PID arithmetic.
  AppC_SetRobotVelocity(2000,std::numeric_limits<float>::quiet_NaN(),0,0,0,0);
  AppC_GetTelemetry(&telem);assert(!telem.communication_ok);near(setpoint_m1,0);
  AppC_SetRobotVelocity(2001,1,0,0,0,0);AppC_FastTick1kHz();near(dbg_motion_limited_xy[0],0.004f);
  AppC_ForceSafeState();near(setpoint_m1,0);
  // Protection must not hide an accumulating ramp that jumps on recovery.
  host_adc=0;
  for(int i=0;i<3;++i) {
    host_tick+=100;AppC_Tick();
    AppC_SetRobotVelocity(2100+i,1,0,0,0,0);AppC_FastTick1kHz();
  }
  AppC_GetTelemetry(&telem);assert(telem.fault_status&0x10);
  near(setpoint_m1,0);assert(cmd_m1==0);assert(dbg_motion_rotation_source==0);
  host_adc=3800;host_tick+=100;AppC_Tick();
  AppC_SetRobotVelocity(2103,1,0,0,0,0);AppC_FastTick1kHz();
  near(dbg_motion_limited_xy[0],0.004f);
  // Actual PID + static feedforward: final saturation must freeze integration
  // in both directions even when the PID alone is still below its rail.
  AppContext context{&adc,&pwm1,&pwm8,&et[0],&et[1],&et[2],&et[3],&gpio,0};
  App app(context);
  for(float sign : {-1.f,1.f}) {
    auto &pid=app.pid1_;pid.integral=sign*2380;pid.lastMeas=sign*100;
    const float output=app.ComputePid(pid,sign*101,sign*100,0.001f,0);
    near(pid.integral,sign*2380);
    near(app.ApplyStaticPwm(sign*101,sign*100,output,0),sign*2399);
    app.ComputePid(pid,0,sign*100,0.001f,0);near(pid.integral,0);
    near(app.ApplyStaticPwm(sign*2.9f,0,2000,0),0);
    near(app.ApplyStaticPwm(sign*10,0,0,0),sign*425);
    near(app.ApplyStaticPwm(sign*10,sign*10,0,0),sign*40);
    pid.integral=sign*1000;pid.lastMeas=sign*200;
    assert(sign*app.ComputePid(pid,-sign*100,sign*200,0.001f,0)<0);
  }
  // Encoder fault trips at 300 ticks and can be cleared by a zero reference.
  for(int i=0;i<300;++i) app.ApplyEncoderFaultProtection(0,app.pid1_,100,0,2000);
  assert(app.FaultStatus()&1);assert(app.ApplyEncoderFaultProtection(0,app.pid1_,100,0,2000)==0);
  app.ApplyEncoderFaultProtection(0,app.pid1_,0,0,0);assert(!(app.FaultStatus()&1));
  Motor motor(&pwm1,0,1);motor.ApplySigned(9999,false);assert(host_tim1.compare[0]==2399);
  motor.ApplySigned(-9999,false);assert(host_tim1.compare[1]==2399 && host_tim1.compare[0]==0);
  motor.ForceStop(true);assert(host_tim1.compare[0]==2399 && host_tim1.compare[1]==2399);
  motor.ForceStop(false);assert(host_tim1.compare[0]==0 && host_tim1.compare[1]==0);
  std::puts("G pipeline: watchdog 150 ms, frozen sequence, restart, NaN, PID rails/signs/anti-windup, encoder wrap/fault, PWM passed");
}
