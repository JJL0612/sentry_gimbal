"""Real follow planner + real GM6020/PID loop, with simulated motors and IMUs."""
from pathlib import Path
import subprocess
import tempfile
root = Path(__file__).resolve().parents[2]
motor = root/'User_File/2_Device/Motor/Motor_DJI'
pid = root/'User_File/1_Middleware/Algorithm/PID'
header = (motor/'dvc_motor_dji_6020.h').read_text()
header = header[header.index('struct Struct_Motor_DJI_GM6020_Position_Control_Config'):header.rindex('#endif')]
source = (motor/'dvc_motor_dji_6020.cpp').read_text()
source = source[source.index('void Class_Motor_DJI_GM6020_Position_Control::Init'):]
task = (root/'User_File/4_Task/tsk_config_and_callback.cpp').read_text()
config = task[task.index('        Struct_Motor_DJI_GM6020_Position_Control_Config gm6020_config'):task.index('        gm6020_ctrl.Init(')]
basic = r'''
#pragma once
#include <algorithm>
#include <cmath>
#include <cstdint>
using std::isfinite;
#define MOTOR_DJI_CTRL_PI 3.14159265358979323846f
inline float Basic_Math_Abs(float a){return std::fabs(a);}
inline float Basic_Math_Constrain(float a,float lo,float hi){return std::clamp(a,lo,hi);}
inline void Basic_Math_Constrain(float *a,float lo,float hi){*a=std::clamp(*a,lo,hi);}
inline float Basic_Math_Modulus_Normalization(float a,float period){return std::remainder(a,period);}
'''
stub = r'''
struct FDCAN_HandleTypeDef {};
enum Enum_Motor_DJI_ID { Motor_DJI_ID_0x20B };
enum {Motor_DJI_Control_Method_TORQUE,Motor_DJI_Status_ENABLE=1};
struct Class_Motor_DJI_GM6020 {
 float angle=0.765f,omega=0,torque=0;
 int status=Motor_DJI_Status_ENABLE;
 void Init(const FDCAN_HandleTypeDef*,Enum_Motor_DJI_ID,int,int,float){}
 float Get_Now_Angle(){return angle;} float Get_Now_Omega(){return omega;}
 float Get_Now_Torque(){return torque;} int Get_Status(){return status;}
 void Set_Target_Torque(float v){torque=v;}
 void TIM_Calculate_PeriodElapsedCallback(){}
};
'''
tests = r'''
#include <cassert>
#include <cstdio>
#include "1_Middleware/Algorithm/IMU/alg_yaw_follow.h"
using yaw_follow::Controller;
using yaw_follow::SmallImu;
static void near(float a,float b,float eps=0.0001f){if(std::fabs(a-b)>=eps)std::fprintf(stderr,"near: %.8f vs %.8f\n",a,b);assert(std::fabs(a-b)<eps);}
static float wrap(float a){return std::remainder(a,2*MOTOR_DJI_CTRL_PI);}
int main(){
 Controller c;
 SmallImu imu{true,1,0,0,0,1};
 auto out=c.update(true,2,0,0,1,true,0.9f,0,100,imu);
 assert(out.valid);near(out.small_target_rad,0.9f);assert(out.big_world_rate_rad_s>0);
 // Correct a real world-angle drift even when encoder and big yaw haven't changed.
 imu.heading_rad=1.01f;
 out=c.update(true,2,0,0,1,true,0.9f,0,110,imu);
 near(out.small_target_rad,0.89f);
 // Absolute zeros differ; changing big-IMU heading alone must not change gun aim.
 out=c.update(true,-2,0,0,1,true,0.9f,0,111,imu);near(out.small_target_rad,0.89f);
 // Zero gun error still requires relative motion to cancel moving big yaw.
 imu.heading_rad=1;out=c.update(true,2,0.6f,0,1,true,0.9f,0,112,imu);
 near(out.relative_rate_feedforward,-0.6f);near(out.measured_relative_rate,-0.6f);
 imu.rate_rad_s=0.1f;out=c.update(true,2,0.6f,0,1,true,0.9f,0,113,imu);
 assert(out.relative_rate_feedforward-out.measured_relative_rate<0);
 // Both external stream ages and validity are mandatory; invalid data resets target.
 for(int f=0;f<7;f++){
  SmallImu bad=imu;
  if(f==0)bad.valid=false;if(f==1)bad.age_ms=51;if(f==2)bad.gyro_age_ms=51;
  if(f==3)bad.heading_rad=NAN;if(f==4)bad.rate_rad_s=NAN;if(f==5)bad.sequence=0;
  out=c.update(true,2,0,0,1,f!=6,0.9f,0,120+f,bad);assert(!out.valid);
  out=c.update(true,2,0,0,1,true,0.9f,0,130+f,imu);assert(out.valid);near(out.small_target_rad,0.9f);
 }
 // Stream restart and IMU generation changes capture current gun heading.
 imu.sequence=100;out=c.update(true,2,0,0,1,true,0.9f,0,200,imu);
 imu.sequence=1;imu.heading_rad=-2;out=c.update(true,2,0,0,1,true,0.9f,0,201,imu);
 near(out.small_target_rad,0.9f);
 imu.heading_rad=3.13f;c.reset();out=c.update(true,0,0,0,1,true,0.765f,0,300,imu);
 imu.heading_rad=-3.13f;out=c.update(true,0,0,0,1,true,0.765f,0,301,imu);
 assert(std::fabs(out.world_error_rad)<0.03f);
 imu.heading_rad=0.1f;imu.rate_rad_s=0.2f;imu.age_ms=20;c.reset();
 out=c.update(true,0,0,0,1,true,0.765f,0,400,imu);near(out.world_error_rad,0);
 imu.age_ms=0;imu.heading_rad=0.104f;out=c.update(true,0,0,0,1,true,0.765f,0,401,imu);
 near(out.world_error_rad,0);
 // Disabling and tick rollover must capture safely without a large integration step.
 imu={true,0,0,0,0,1};c.reset();
 out=c.update(true,0,0,0,1,true,0.765f,0,UINT32_MAX-4,imu);
 out=c.update(true,0,0,0,1,true,0.765f,0.5f,5,imu);near(out.small_target_rad,0.770f);
 out=c.update(false,0,0,0,1,true,0.765f,0,6,imu);assert(!out.valid);
 // A blocked big yaw cannot accumulate world targets beyond small joint travel.
 c.reset();float blocked_small=0.765f;
 for(unsigned i=0;i<10000;i++){
  imu={true,blocked_small-0.765f,0,0,0,i+1};
  out=c.update(true,0,0,0,1,true,blocked_small,1,i,imu);
  assert(out.valid && out.small_target_rad<=1.53f);blocked_small=out.small_target_rad;
 }
 imu={true,blocked_small-0.765f,0,0,0,10001};
 out=c.update(true,0,0,0,1,true,blocked_small,-1,10000,imu);
 assert(out.small_target_rad<1.53f);
 const bool BIG_YAW_FOLLOW_SMALL_ENABLE=true;
 const float GM6020_POSITION_MIN_RAD=0,GM6020_POSITION_MAX_RAD=1.53f;
'''
tests += config + r'''
 // Real velocity loop must not brake by zero world-angle error when the joint must move.
 Class_Motor_DJI_GM6020_Position_Control servo;
 servo.Init(nullptr,Motor_DJI_ID_0x20B,0,gm6020_config);
 servo.Set_Sample_Target_Enable(false);servo.Set_Target_Angle_Manually(0.765f);
 servo.Set_Inertial_Velocity(true,-1,0);
 for(int i=0;i<50;i++)servo.TIM_1ms_PeriodElapsedCallback(i);
 assert(servo.Velocity_Reference_RadS<-0.9f && servo.Target_Current_A<0);
 // Both hard limits block outward current, while offline/disabled states output zero.
 for(float edge:{0.0f,1.53f}){
  servo.Motor.angle=edge;servo.Set_Target_Angle_Manually(edge);
  servo.Set_Inertial_Velocity(true,edge==0?-1:1,0);
  for(int i=0;i<50;i++)servo.TIM_1ms_PeriodElapsedCallback(i);
  assert(edge==0 ? servo.Target_Current_A>=0 : servo.Target_Current_A<=0);
 }
 servo.Set_Output_Enable(false);for(int i=0;i<5;i++)servo.TIM_1ms_PeriodElapsedCallback(i);
 near(servo.Motor.torque,0);
 servo.Set_Output_Enable(true);servo.Motor.status=0;
 for(int i=0;i<5;i++)servo.TIM_1ms_PeriodElapsedCallback(i);
 near(servo.Motor.torque,0);servo.Motor.status=Motor_DJI_Status_ENABLE;
 // Full planner + actual cascaded PID, delayed external samples, motor friction,
 // small torque disturbance and a moving big yaw. No duplicated servo equations.
 for(float direction:{-1.0f,1.0f}){
  c.reset();servo.Init(nullptr,Motor_DJI_ID_0x20B,0,gm6020_config);
  servo.Set_Sample_Target_Enable(false);
  float big=0,small=0.765f,gun_heading=0,gun_rate=0,big_rate=0,gun_sample=0,rate_sample=0;
  float peak_error=0,max_offset=0;
  for(unsigned i=0;i<12000;i++){
   const float stick=i<600?direction*0.5f:0;
   // Big yaw follows the measured small offset, with 20ms actuator lag and
   // residual base-spin ripple after its independent stabilization loop.
   const float follow=std::fabs(small-0.765f)>0.01f?std::clamp(2*(small-0.765f),-1.0f,1.0f):0;
   big_rate+=0.001f/0.020f*(follow+0.06f*std::sin(2*MOTOR_DJI_CTRL_PI*i*0.001f/4)-big_rate);
   if(i%6==0){gun_sample=gun_heading;rate_sample=gun_rate;}
   SmallImu measured{true,gun_sample,rate_sample,i%6,i%6,i/6+1};
   out=c.update(true,big,big_rate,0,1,true,small,stick,i+1000,measured);assert(out.valid);
   servo.Motor.angle=small;servo.Motor.omega=gun_rate-big_rate;
   servo.Set_Target_Angle_Manually(out.small_target_rad);
   servo.Set_Inertial_Velocity(true,out.relative_rate_feedforward,out.measured_relative_rate);
   servo.TIM_1ms_PeriodElapsedCallback(i);
   assert(std::fabs(servo.Motor.torque)<=gm6020_config.Current_Max_A*0.741f+0.0001f);
   gun_rate+=0.001f*(servo.Motor.torque/0.02f-0.1f*(gun_rate-big_rate)+0.01f);
   big=wrap(big+big_rate*0.001f);gun_heading=wrap(gun_heading+gun_rate*0.001f);
   small+=0.001f*(gun_rate-big_rate);
   assert(small>0&&small<1.53f);
   max_offset=std::max(max_offset,std::fabs(small-0.765f));
   if(i>1000)peak_error=std::max(peak_error,std::fabs(wrap(gun_heading-direction*0.2995f)));
  }
  std::printf("Small-yaw simulated world error: final %.5f rad, peak after release %.5f rad\n",
   std::fabs(wrap(gun_heading-direction*0.2995f)),peak_error);
  assert(max_offset>0.05f);
  assert(std::fabs(wrap(gun_heading-direction*0.2995f))<0.01f);
  assert(peak_error<0.02f);
  assert(std::fabs(small-0.765f)<0.06f);
 }
 puts("PASS: external world feedback, gyro feedforward, real GM6020/PID loop, recenter, limits, stream faults and restart");
}
'''
with tempfile.TemporaryDirectory(prefix='yaw_follow_') as directory:
    work=Path(directory)
    (work/'basic_stub.h').write_text(basic)
    (work/'alg_pid.h').write_text((pid/'alg_pid.h').read_text().replace('"1_Middleware/Algorithm/Basic/alg_basic.h"','"basic_stub.h"'))
    (work/'alg_pid.cpp').write_text((pid/'alg_pid.cpp').read_text())
    (work/'test.cpp').write_text('#include "alg_pid.h"\n'+stub+header+source+tests)
    subprocess.run(['g++','-std=c++17','-Wall','-Wextra','-Werror','-Wno-misleading-indentation',
                    '-fsanitize=address,undefined','-I'+str(root/'User_File'),'-I'+str(work),
                    str(work/'test.cpp'),str(work/'alg_pid.cpp'),'-o',str(work/'test')],check=True)
    subprocess.run([str(work/'test')],check=True)
