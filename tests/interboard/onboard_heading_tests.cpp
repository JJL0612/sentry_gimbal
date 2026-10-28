#include "1_Middleware/Algorithm/IMU/alg_onboard_heading.h"
#include "2_Device/Communication/dvc_chassis_link.h"
extern "C" {
#include "app_gimbal_link.h"
}
#include <cassert>
#include <cmath>
#include <iostream>

static std::uint32_t now;
extern "C" std::uint32_t HAL_GetTick(void) {return now;}
static void near(float a,float b,float eps=0.0001f){assert(std::fabs(a-b)<eps);}
static unsigned irq_mask=0;
static unsigned __get_PRIMASK(){return irq_mask;}
static void __disable_irq(){irq_mask=1;}
static void __set_PRIMASK(unsigned mask){irq_mask=mask;}
static gimbal_imu::HeadingOrigin onboard_origin;
static struct MockBoard {
 gimbal_imu::HeadingSample value;
 gimbal_imu::HeadingSample Get_Heading_Sample(){assert(irq_mask==1);return value;}
} BSP_BMI088;
static struct MockClock {
 std::uint64_t Get_Now_Microsecond(){assert(irq_mask==1);return std::uint64_t(now)*1000;}
} SYS_Timestamp;
#include "onboard_fill_input.inc"
int main(){
 constexpr float g=9.80665f,pi=3.14159265358979323846f;
 // Geometry is deliberately independent of compensation constants in the implementation.
 for(float w: {-4.0f,0.0f,4.0f}){
  gimbal_imu::PlanarLeverArm arm;arm.observe_rate(w,1000);
  const auto a=arm.correct({0.0436f*w*w,0.04996f*w*w,g});
  near(a.x,0);near(a.y,0);near(a.z,g);
 }
 gimbal_imu::PlanarLeverArm arm;
 arm.observe_rate(0,1000);
 for(unsigned i=1;i<=2000;i++){
  const float w=i*0.001f*0.8f;
  arm.observe_rate(w,1000+i*1000);
  const auto a=arm.correct({0.04996f*0.8f+0.0436f*w*w,
                           -0.0436f*0.8f+0.04996f*w*w,g});
  if(i>500){near(a.x,0,0.0002f);near(a.y,0,0.0002f);}
 }
 near(arm.alpha(),0.8f,0.001f);
 arm.observe_rate(10,2001000);near(arm.alpha(),0.8f,0.001f); // same sample is not a new derivative
 arm.observe_rate(0,2101000);near(arm.alpha(),0); // no differentiation across a gap
 gimbal_imu::HeadingOrigin origin;origin.init(0);
 gimbal_imu::HeadingSample s;
 s.healthy=true;s.generation=1;s.yaw_rad=179*pi/180;s.accel_norm_mps2=g;
 gimbal_imu::HeadingOutput h;
 auto update=[&](unsigned t){s.sample_us=std::uint64_t(t)*1000;s.sequence++;return origin.update(s,s.sample_us+500,t);};
 for(unsigned t=10;t<1000;t+=5)assert(!update(t).valid);
 update(1000);
 // Repeated foreground reads of one sample cannot complete startup zeroing.
 for(int i=0;i<50;i++)assert(!origin.update(s,s.sample_us+500,1000).valid);
 for(unsigned t=1005;t<=1095;t+=5)h=update(t);
 assert(h.valid);near(h.delta_rad,0);assert(h.generation==1);
 s.yaw_rad=-179*pi/180;s.yaw_rate_rad_s=1.25f;h=update(1100);
 near(h.delta_rad,2*pi/180,0.0002f);near(h.rate_rad_s,1.25f);
 // Application does not integrate rate: identical EKF angles must give identical deltas.
 h=update(1105);near(h.delta_rad,2*pi/180,0.0002f);
 assert(!origin.update(s,s.sample_us+51000,1156).valid);
 s.yaw_rad=-178*pi/180;h=update(1160);assert(h.valid);near(h.delta_rad,3*pi/180,0.0002f);
 s.healthy=false;assert(!update(1165).valid);s.healthy=true;
 s.yaw_rad=NAN;assert(!update(1170).valid);s.yaw_rad=-178*pi/180;
 h=update(1175);assert(h.valid);near(h.delta_rad,3*pi/180,0.0002f);
 // EKF reinitialization is communicated as a different coordinate generation.
 s.generation=2;s.yaw_rate_rad_s=0;s.yaw_rad=0.5f;
 assert(!update(1180).valid);
 for(unsigned t=1185;t<=1275;t+=5)h=update(t);
 assert(h.valid&&h.generation==2);near(h.delta_rad,0);
 // Native EKF heading -> existing UART encoder -> actual chassis receiver.
 now=1275;GimbalLinkInit();chassis_link::Link link;link.init(123);
 BoardLinkHeartbeat hb{};hb.role=kBoardLinkRoleChassisController;hb.sender_boot_id=GimbalLinkLocalBootId();
 hb.safety_state=kBoardLinkSafetyWaiting;
 std::uint8_t packet[256];std::uint16_t size=0;
 assert(BoardLinkEncodeHeartbeat(&hb,1,packet,sizeof(packet),&size));link.consume(packet,size,now);
 chassis_link::Input in{};in.remote_online=true;in.left_switch=-1;in.remote_sequence=1;
 onboard_origin.init(0);
 BSP_BMI088.value=s;
 for(unsigned i=1;i<=20;i++){
  now=1275+i;BSP_BMI088.value.sequence=i;
  BSP_BMI088.value.sample_us=std::uint64_t(now)*1000;
  Dm_Imu_FillInput(in);assert(irq_mask==0);
 }
 assert(in.imu_valid&&in.imu_present&&in.imu_zero_generation==2);
 link.update(in,now);in.left_switch=0;link.update(in,now);
 size=link.encode(packet,sizeof(packet),now);assert(size==126);GimbalLinkRxCallback(packet,size);
 GimbalImuYaw got{};assert(GimbalLinkGetImuYaw(&got));
 assert(got.zero_generation==2&&got.source_sequence==in.imu_sequence);near(got.yaw_delta_rad,in.imu_yaw_delta_rad);
 in.imu_valid=false;link.update(in,++now);size=link.encode(packet,sizeof(packet),now);
 GimbalLinkRxCallback(packet,size);assert(!GimbalLinkGetImuYaw(&got));
 std::cout<<"PASS: onboard right-rear lever arm, measured rate derivative, direct EKF angles, one boot zero, stale/reinit handling and UART generation transport\n";
}
