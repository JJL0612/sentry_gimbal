#include "2_Device/IMU/dvc_dm_imu.h"
#include "2_Device/Communication/dvc_chassis_link.h"
extern "C" {
#include "app_gimbal_link.h"
}
#include <cassert>
#include <cmath>
#include <cstring>
#include <iostream>
#include <limits>

static std::uint32_t now;
extern "C" std::uint32_t HAL_GetTick(void) { return now; }
static void near(float a,float b,float epsilon=0.0001f){assert(std::fabs(a-b)<epsilon);}
static void crc(std::uint8_t *p,unsigned n){auto v=chassis_link::crc16(p,n-2);p[n-2]=v;p[n-1]=v>>8;}
static void vector_frame(dm_imu::Sensor &imu,unsigned type,dm_imu::Vector3 v,float limit,std::uint32_t time){
 std::uint8_t p[8]={std::uint8_t(type),std::uint8_t(type==1?25:0),0,0,0,0,0,0};
 const float values[]={v.x,v.y,v.z};
 for(unsigned i=0;i<3;i++){
  const auto raw=unsigned(std::lround((values[i]+limit)/(2*limit)*65535));
  p[2+2*i]=raw;p[3+2*i]=raw>>8;
 }
 assert(imu.consume(0x11,p,8,time));
}
static void sensor_frame(dm_imu::Sensor &imu, float yaw, float rate, std::uint32_t time){
 auto y=unsigned(std::lround((yaw+180)/360*65535));
 vector_frame(imu,2,{0,0,rate},34.88f,time);
 vector_frame(imu,1,{-dm_imu::lever_forward_m*rate*rate,-dm_imu::lever_left_m*rate*rate,9.80665f},235.2f,time);
 std::uint8_t euler[8]={3,0,0,0,std::uint8_t(y),std::uint8_t(y>>8),0,0};
 assert(imu.consume(0x11,euler,8,time));
}
static void compensation_tests(){
 constexpr float g=9.80665f,dt=0.006f,pi=3.14159265358979323846f;
 const float rx=dm_imu::lever_forward_m,ry=dm_imu::lever_left_m;
 dm_imu::GimbalAhrs a;
 // Independent circular-motion construction: 4 rad/s induces ~2 m/s^2 at this arm.
 const dm_imu::Vector3 circular{-16*rx,-16*ry,g};
 assert(a.initialize(circular,{0,0,4},179*pi/180));
 for(unsigned i=0;i<1000;i++)assert(a.update(circular,{0,0,4},dt));
 auto corrected=a.corrected_accel();near(corrected.x,0);near(corrected.y,0);near(corrected.z,g);
 near(a.pitch(),0,0.0001f);near(a.roll(),0,0.0001f);
 near(std::remainder(a.yaw()-(179*pi/180+4*dt*1000),2*pi),0,0.002f);
 // Measured rate ramp, including both tangential and centripetal acceleration.
 assert(a.initialize({0,0,g},{0,0,0},0));
 float heading=0;
 for(unsigned i=1;i<=1000;i++){
  const float w=0.8f*i*dt,alpha=0.8f;
  const dm_imu::Vector3 raw{-alpha*ry-w*w*rx,alpha*rx-w*w*ry,g};
  assert(a.update(raw,{0,0,w},dt));heading+=w*dt;
 }
 near(a.yaw_acceleration(),0.8f,0.001f);
 near(a.corrected_accel().x,0,0.0002f);near(a.corrected_accel().y,0,0.0002f);
 near(a.roll(),0,0.002f);near(a.pitch(),0,0.002f);
 near(std::remainder(a.yaw()-heading,2*pi),0,0.003f);
 // Gravity feedback must converge rather than amplify roll error.
 assert(a.initialize({0,g*std::sin(0.2f),g*std::cos(0.2f)},{},0));
 for(unsigned i=0;i<2000;i++)assert(a.update({0,0,g},{},dt));
 assert(std::fabs(a.roll())<0.02f);
 const float yaw=a.yaw();assert(!a.update({0,0,g},{},0));
 assert(!a.update({0,0,g},{},0.051f));
 assert(!a.update({NAN,0,g},{},dt));near(a.yaw(),yaw);
 // A large linear acceleration must not be interpreted as gravity.
 assert(a.initialize({0,0,g},{},0));
 for(unsigned i=0;i<200;i++)assert(a.update({10,0,g},{},dt));
 near(a.pitch(),0);
}
static void recorded_hardware_tests(){
 // First real CAN3 sample read through CMSIS-DAP on 2026-10-28.
 // Acceleration temperature byte is 25; acceleration unit is m/s^2.
 const std::uint8_t frames[][8]={
  {2,0,0x06,0x80,0x0e,0x80,0xfc,0x7f},
  {1,25,0x04,0x80,0xdf,0x7f,0x42,0x85},
  {3,0,0x2a,0x80,0x01,0x80,0x24,0x7f}
 };
 dm_imu::Sensor imu;imu.init(0);
 for(unsigned i=0;i<250;i++)for(unsigned j=0;j<3;j++)
  assert(imu.consume(0x11,frames[j],8,i*6+j));
 auto s=imu.snapshot(1496);assert(s.valid&&s.zero_valid);
 near(s.yaw_delta_rad,0,0.0002f);near(s.yaw_rate_rad_s,0,0.00001f);
 near(s.corrected_accel_mps2.x,s.raw_accel_mps2.x);
 near(s.roll_rad,std::atan2(-0.2332799f,9.664967f),0.0001f);
 // Startup motion rejection also includes X/Y gyro; yaw alone is not sufficient.
 dm_imu::Sensor tilting;tilting.init(0);
 for(unsigned i=0;i<240;i++){
  vector_frame(tilting,2,{0.2f,0,0},34.88f,i*6);
  vector_frame(tilting,1,{0,0,9.80665f},235.2f,i*6+1);
  tilting.consume(0x11,frames[2],8,i*6+2);
 }
 assert(!tilting.snapshot(1436).zero_valid);
 // No startup re-zero after losing/recovering data; recover measured heading movement in the gap.
 const auto zero=s.yaw_zero_rad;
 sensor_frame(imu,45,0,1600);sensor_frame(imu,45,0,1606);
 s=imu.snapshot(1606);assert(s.valid);near(s.yaw_zero_rad,zero);
 near(s.yaw_delta_rad,45*3.14159265358979323846f/180-zero,0.0002f);
}
static void heartbeat(chassis_link::Link &link,std::uint32_t seq=1){
 BoardLinkHeartbeat h{};h.role=kBoardLinkRoleChassisController;h.sender_boot_id=GimbalLinkLocalBootId();
 h.safety_state=kBoardLinkSafetyWaiting;
 std::uint8_t b[64];std::uint16_t n=0;assert(BoardLinkEncodeHeartbeat(&h,seq,b,sizeof(b),&n));
 link.consume(b,n,now);
}
int main(){
 compensation_tests();
 recorded_hardware_tests();
 dm_imu::Sensor imu;imu.init(0);
 for(unsigned i=0;i<30;i++)sensor_frame(imu,179.0f,0,i*5);
 assert(!imu.snapshot(150).zero_valid);
 for(unsigned i=0;i<20;i++)sensor_frame(imu,179.0f,0,1000+i*5);
 sensor_frame(imu,179.0f,0,1100);
 auto s=imu.snapshot(1100);assert(s.valid&&s.zero_valid&&s.feedback_id==0x11);
 near(s.yaw_delta_rad,0);
 sensor_frame(imu,-179.0f,0,1105);s=imu.snapshot(1105);
 // Vendor Euler is for boot/gap alignment; a live Euler jump must not overwrite integrated heading.
 near(s.yaw_delta_rad,0,0.0002f);
 near(s.raw_accel_mps2.z,9.80665f,0.004f);
 near(s.corrected_accel_mps2.x,s.raw_accel_mps2.x);
 assert(imu.snapshot(1116).valid); // tolerate a single missed CAN reply
 assert(!imu.snapshot(1156).valid);
 sensor_frame(imu,-178.0f,0,1158);
 sensor_frame(imu,-178.0f,0,1160);s=imu.snapshot(1160);assert(s.valid);
 near(s.yaw_zero_rad,179*3.14159265358979323846f/180,0.0002f);
 std::uint8_t wrong[8]={3};assert(!imu.consume(0x12,wrong,8,1161));
 assert(!imu.consume(0x11,wrong,7,1161));
 // Actual captured CAN acceleration frame: temperature byte is not a reserved zero.
 std::uint8_t measured[8]={1,25,0x04,0x80,0xdf,0x7f,0x42,0x85};
 assert(imu.consume(0x11,measured,8,1160));
 auto measured_s=imu.snapshot(1160);
 near(measured_s.raw_accel_mps2.x,0.0323003f,0.00003f);
 near(measured_s.raw_accel_mps2.y,-0.2332799f,0.00003f);
 near(measured_s.raw_accel_mps2.z,9.664967f,0.00003f);
 dm_imu::Sensor moving;moving.init(0);
 for(unsigned i=0;i<100;i++)sensor_frame(moving,10,1,1000+i*5);
 assert(!moving.snapshot(1495).zero_valid);
 // Without acceleration, the IMU cannot silently publish uncompensated valid heading.
 dm_imu::Sensor missing;missing.init(0);
 for(unsigned i=0;i<40;i++){
  vector_frame(missing,2,{},34.88f,1000+i*6);
  std::uint8_t e[8]={3,0,0,0,0,0x80,0,0};missing.consume(0x11,e,8,1000+i*6);
 }
 assert(!missing.snapshot(1234).valid&&!missing.snapshot(1234).zero_valid);
 for(unsigned slot=0;slot<9;slot++){
  std::uint16_t id;std::uint8_t b[8];assert(dm_imu::request(slot,id,b)==8);
  assert(id==1&&b[0]==0xcc&&b[1]==(slot%3==0?2:slot%3==1?1:3)&&b[2]==0&&b[3]==0xdd);
  for(unsigned i=4;i<8;i++)assert(b[i]==0); // no parameter writes
 }
 // Actual IMU -> actual gimbal encoder -> actual chassis parser/consumer.
 now=1160;GimbalLinkInit();chassis_link::Link link;link.init(123);heartbeat(link);
 chassis_link::Input in{};in.remote_online=true;in.left_switch=-1;in.remote_sequence=1;
 in.imu_present=true;in.imu_valid=s.valid;in.imu_yaw_delta_rad=s.yaw_delta_rad;
 in.imu_yaw_rate_rad_s=s.yaw_rate_rad_s;in.imu_sequence=s.sequence;
 link.update(in,now);in.left_switch=0;in.wz=0.8f;link.update(in,now);
 std::uint8_t batch[chassis_link::batch_size];
 auto emit=[&](){auto n=link.encode(batch,sizeof(batch),now);assert(n==126);GimbalLinkRxCallback(batch,n);};
 emit();GimbalImuYaw yaw{};assert(GimbalLinkGetImuYaw(&yaw));
 near(yaw.yaw_delta_rad,s.yaw_delta_rad);near(yaw.yaw_rate_rad_s,s.yaw_rate_rad_s);
 assert(yaw.sender_boot_id==123&&yaw.source_sequence==s.sequence&&yaw.zero_generation==1);
 BoardLinkChassisControl c{};assert(GimbalLinkGetChassisControl(&c)&&c.global_action==2);near(c.wz_rad_s,0.8f);
 // Replaying a frame must not refresh the IMU snapshot.
 now+=30;GimbalLinkRxCallback(batch,126);assert(GimbalLinkGetImuYaw(&yaw));
 now+=21;assert(!GimbalLinkGetImuYaw(&yaw));
 // A producer replaying a sensor sequence with a new UART frame also cannot keep it fresh.
 link.update(in,now);emit();assert(!GimbalLinkGetImuYaw(&yaw));
 // New sensor sample restores availability, retaining source age.
 in.imu_sequence++;in.imu_sample_age_ms=10;link.update(in,now);emit();
 assert(GimbalLinkGetImuYaw(&yaw)&&yaw.source_age_ms==10);
 // Wrong receiver, CRC corruption, and invalid numeric values cannot replace valid data.
 auto original=yaw.source_sequence;
 in.imu_sequence++;link.update(in,now);link.encode(batch,sizeof(batch),now);
 std::uint8_t *msg=batch+80;msg[12+24]^=1;crc(msg,46);
 GimbalLinkRxCallback(msg,46);assert(GimbalLinkGetImuYaw(&yaw)&&yaw.source_sequence==original);
 msg[12+24]^=1;float nan=std::numeric_limits<float>::quiet_NaN();std::memcpy(msg+16,&nan,4);crc(msg,46);
 GimbalLinkRxCallback(msg,46);assert(GimbalLinkGetImuYaw(&yaw)&&yaw.source_sequence==original);
 // Explicit sensor invalidity revokes immediately, even when controls remain active.
 in.imu_valid=false;link.update(in,now);emit();assert(!GimbalLinkGetImuYaw(&yaw));
 // Sender restart must flush the previous sensor snapshot.
 in.imu_valid=true;in.imu_sequence++;link.update(in,now);emit();assert(GimbalLinkGetImuYaw(&yaw));
 chassis_link::Link reboot;reboot.init(124);heartbeat(reboot,2);reboot.update({},now);
 auto n=reboot.encode(batch,sizeof(batch),now);GimbalLinkRxCallback(batch,n);
 assert(!GimbalLinkGetImuYaw(&yaw));
 // The roll wheel uses the independent standard BigYawRequest path, without touching chassis wz.
 now+=1;heartbeat(link,3);in.big_yaw_present=true;in.big_yaw_rate_rad_s=0.6f;
 in.remote_sequence++;in.wz=0;link.update(in,now);
 auto size=link.encode(batch,sizeof(batch),now);assert(size==176);GimbalLinkRxCallback(batch,size);
 BoardLinkBigYawRequest request{};assert(GimbalLinkGetBigYawRequest(&request));
 assert(request.permission_valid&&request.permission_enabled);near(request.target_rate_rad_s,0.6f);
 assert(GimbalLinkGetChassisControl(&c));near(c.wz_rad_s,0);
 in.remote_sequence++;in.wz=0.8f;in.big_yaw_rate_rad_s=-0.7f;link.update(in,now);
 size=link.encode(batch,sizeof(batch),now);GimbalLinkRxCallback(batch,size);
 assert(GimbalLinkGetBigYawRequest(&request));near(request.target_rate_rad_s,-0.7f);
 assert(GimbalLinkGetChassisControl(&c));near(c.wz_rad_s,0.8f);
 in.remote_online=false;link.update(in,now);size=link.encode(batch,sizeof(batch),now);GimbalLinkRxCallback(batch,size);
 assert(GimbalLinkGetBigYawRequest(&request)&&!request.permission_enabled);near(request.target_rate_rad_s,0);
 std::cout<<"PASS: hardware-frame replay, lever-arm circular/ramp motion, Mahony feedback, startup bias, dropout recovery, read-only queries, v4 controls and transport\n";
}
