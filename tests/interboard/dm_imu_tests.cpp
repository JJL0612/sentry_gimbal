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
static void sensor_frame(dm_imu::Sensor &imu, float yaw, float rate, std::uint32_t time){
 auto y=unsigned(std::lround((yaw+180)/360*65535));
 auto g=unsigned(std::lround((rate+34.88f)/69.76f*65535));
 std::uint8_t gyro[8]={2,0,0,0,0,0,std::uint8_t(g),std::uint8_t(g>>8)};
 std::uint8_t euler[8]={3,0,0,0,std::uint8_t(y),std::uint8_t(y>>8),0,0};
 assert(imu.consume(0x11,gyro,8,time));assert(imu.consume(0x11,euler,8,time));
}
static void heartbeat(chassis_link::Link &link,std::uint32_t seq=1){
 BoardLinkHeartbeat h{};h.role=kBoardLinkRoleChassisController;h.sender_boot_id=GimbalLinkLocalBootId();
 h.safety_state=kBoardLinkSafetyWaiting;
 std::uint8_t b[64];std::uint16_t n=0;assert(BoardLinkEncodeHeartbeat(&h,seq,b,sizeof(b),&n));
 link.consume(b,n,now);
}
int main(){
 dm_imu::Sensor imu;imu.init(0);
 for(unsigned i=0;i<30;i++)sensor_frame(imu,179.0f,0,i*5);
 assert(!imu.snapshot(150).zero_valid);
 for(unsigned i=0;i<20;i++)sensor_frame(imu,179.0f,0,1000+i*5);
 auto s=imu.snapshot(1095);assert(s.valid&&s.zero_valid&&s.feedback_id==0x11);
 near(s.yaw_delta_rad,0);
 sensor_frame(imu,-179.0f,0,1100);s=imu.snapshot(1100);
 near(s.yaw_delta_rad,2*3.14159265358979323846f/180,0.0002f);
 assert(!imu.snapshot(1151).valid);
 sensor_frame(imu,-178.0f,0,1160);s=imu.snapshot(1160);assert(s.valid);
 near(s.yaw_zero_rad,179*3.14159265358979323846f/180,0.0002f);
 std::uint8_t wrong[8]={3};assert(!imu.consume(0x12,wrong,8,1161));
 assert(!imu.consume(0x11,wrong,7,1161));
 dm_imu::Sensor moving;moving.init(0);
 for(unsigned i=0;i<100;i++)sensor_frame(moving,10,1,1000+i*5);
 assert(!moving.snapshot(1495).zero_valid);
 for(unsigned slot=0;slot<4;slot++){
  std::uint16_t id;std::uint8_t b[8];assert(dm_imu::request(slot,id,b)==8);
  assert(id==1&&b[0]==0xcc&&b[1]==((slot&1)?2:3)&&b[2]==0&&b[3]==0xdd);
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
 std::cout<<"PASS: DM IMU real frames, read-only queries, startup zero, yaw wrap, expiry, v4 end-to-end transport, replay, invalidation and restart\n";
}
