"""Exercise actual task remote mapping and actual chassis permission state machine."""
from pathlib import Path
import re
import subprocess
import tempfile

root = Path(__file__).resolve().parents[2]
source = (root / 'User_File/4_Task/tsk_config_and_callback.cpp').read_text()
start = source.index('#if REMOTE_PROTOCOL_SBUS', source.index('// 两侧三档拨杆直接控制电机使能:'))
end = source.index('    Chassis_Link_Tick(chassis_input);', start) + len('    Chassis_Link_Tick(chassis_input);')
body = source[start:end]
constant = '\n'.join(re.search(r'const (?:float|uint32_t|bool) '+name+r' = .*?;', source).group() for name in ('CHASSIS_SPIN_TARGET_OMEGA_RAD_S','CHASSIS_SPIN_PERIOD_MS','BIG_YAW_FOLLOW_SMALL_ENABLE','SMALL_YAW_REMOTE_DIRECTION'))
helper = re.search(r'^static void SmallYawFollow_Update\(.*?^}', source, re.M | re.S).group()
code = r'''
#include "4_Task/tsk_communication.h"
#include <cassert>
#include <cmath>
#include "1_Middleware/Algorithm/IMU/alg_yaw_follow.h"
#include "2_Device/IMU/dvc_dm_imu.h"
using std::fabs;
#include <cstdint>
#include <cstdio>
#define REMOTE_PROTOCOL_SBUS 0
#define PI 3.14159265358979323846f
#define GM6020_REMOTE_TARGET_CHANNEL 0
enum { DR16_Switch_Status_UP, DR16_Switch_Status_DOWN, DR16_Switch_Status_MIDDLE };
struct RemoteMock {
 int left=DR16_Switch_Status_MIDDLE, right=DR16_Switch_Status_MIDDLE;
 int wheel=0, forward=0, sideways=0, aim=0;
 int Get_Switch_Left() {return left;} int Get_Switch_Right() {return right;}
 unsigned Get_Valid_Frame_Count() {return 1;}
 int Get_Channel_Centered(int i) {return i==0 ? aim : (i==2 ? forward : sideways);}
 int Get_Wheel() {return wheel;}
} Remote;
bool remote_controllable=true, joint_running_enable, motor_running_enable;
UART_HandleTypeDef huart1, huart3, huart7, huart10;
uint32_t tick=1;
uint32_t HAL_GetTick(){return tick;}
void UART_Init(UART_HandleTypeDef *, UART_Callback){}
HAL_StatusTypeDef HAL_UART_Transmit_DMA(UART_HandleTypeDef *, uint8_t *, uint16_t){return HAL_OK;}
chassis_link::Input captured;
chassis_link::Link link;
auto &Chassis_Link=link;
bool chassis_link_boot_valid=true, small_yaw_follow_output_enabled=false;
bool small_yaw_feedback_received=true;
uint32_t small_yaw_feedback_ms=1;
float remote_target_angle;
yaw_follow::Controller small_yaw_follow;
float small_angle=0.765f;
bool imu_valid=true, external_valid=true;
struct {
 dm_imu::Snapshot snapshot(uint32_t) {
  dm_imu::Snapshot s{};s.world_valid=external_valid;s.world_sequence=1;return s;
 }
} Gimbal_DM_IMU;
struct GmMock {
 struct {float Get_Now_Angle(){return small_angle;}} Motor;
 void Set_Inertial_Velocity(bool,float,float){}
 void Set_Sample_Target_Enable(bool){}
 void Set_Target_Angle_Manually(float){}
} gm6020_ctrl;
void Dm_Imu_FillInput(chassis_link::Input &in){in.imu_valid=imu_valid;in.imu_zero_generation=1;}

void Dm_Imu_Tick(){}
void Chassis_Link_Tick(const chassis_link::Input &in) {captured=in;link.update(in,1);}
void heartbeat() {
 std::uint8_t b[34]={0xa5,0x5a,4,2,1,0,20,0,1,0,0,0};
 b[12]=2;b[13]=1;b[24]=42;
 auto crc=chassis_link::crc16(b,32);b[32]=crc&255;b[33]=crc>>8;
 link.consume(b,34,1);
}
void near(float a,float b){assert(std::fabs(a-b)<1e-6f);}
'''
code += constant + '\n' + helper + '\nvoid mapping(){\nsmall_yaw_feedback_ms=tick;\n' + body + '\n}\n'
code += r'''
int main(){
 for(int l=0;l<3;l++) for(int r=0;r<3;r++) {
  link.init(123);heartbeat();
  Remote.left=DR16_Switch_Status_DOWN;Remote.right=r;mapping();
  Remote.left=l;Remote.wheel=-660;Remote.forward=330;Remote.sideways=-330;mapping();
  assert(motor_running_enable==(r==DR16_Switch_Status_UP));
  assert(joint_running_enable==(l==DR16_Switch_Status_UP));
  assert(link.enabled()==(l!=DR16_Switch_Status_DOWN));
  near(captured.wz,l==DR16_Switch_Status_UP && r==DR16_Switch_Status_DOWN?3.4f:0.0f);
  assert(captured.big_yaw_present);near(captured.big_yaw_rate_rad_s,0);
  near(captured.vx,l==DR16_Switch_Status_MIDDLE?0:-0.15f);near(captured.vy,l==DR16_Switch_Status_MIDDLE?0:0.15f);
  Remote.forward=-330;mapping();near(captured.vx,l==DR16_Switch_Status_MIDDLE?0:0.15f);near(captured.vy,l==DR16_Switch_Status_MIDDLE?0:0.15f);
  Remote.forward=330;
  Remote.wheel=660;mapping();near(captured.big_yaw_rate_rad_s,0);
  near(captured.wz,l==DR16_Switch_Status_UP && r==DR16_Switch_Status_DOWN?3.4f:0.0f);
  Remote.right=DR16_Switch_Status_MIDDLE;Remote.wheel=0;mapping();
  assert(!motor_running_enable);near(captured.wz,0);near(captured.big_yaw_rate_rad_s,0);
  Remote.right=DR16_Switch_Status_DOWN;remote_controllable=false;mapping();
  assert(!motor_running_enable&&!joint_running_enable&&!link.enabled());
  near(captured.wz,0);near(captured.vx,0);near(captured.vy,0);near(captured.big_yaw_rate_rad_s,0);
  remote_controllable=true;
 }
 link.init(123);heartbeat();Remote.left=DR16_Switch_Status_MIDDLE;
 Remote.right=DR16_Switch_Status_DOWN;mapping();assert(!link.enabled());
 // Exercise the actual wrapper's direction, sensor gates and wheel disconnection.
 tick=1;link.init(123);heartbeat();Remote.left=DR16_Switch_Status_DOWN;mapping();
 Remote.left=DR16_Switch_Status_UP;mapping();mapping();
 assert(captured.big_yaw_permission_enabled && small_yaw_follow_output_enabled);
 small_angle=0.9f;mapping();assert(captured.big_yaw_rate_rad_s>0);
 const float following_rate=captured.big_yaw_rate_rad_s;
 Remote.wheel=-660;mapping();near(captured.big_yaw_rate_rad_s,following_rate);
 Remote.wheel=660;mapping();near(captured.big_yaw_rate_rad_s,following_rate);
 // Horizontal stick direction reverses only the operator command.
 small_angle=0.765f;Remote.aim=0;mapping();
 const float before_aim=remote_target_angle;
 Remote.aim=330;tick+=10;mapping();assert(remote_target_angle<before_aim);
 Remote.aim=0;
 imu_valid=false;mapping();assert(!captured.big_yaw_permission_enabled&&!small_yaw_follow_output_enabled);
 imu_valid=true;small_angle=0.765f;mapping();assert(captured.big_yaw_permission_enabled);
 external_valid=false;mapping();assert(!captured.big_yaw_permission_enabled&&!small_yaw_follow_output_enabled);
 external_valid=true;mapping();assert(captured.big_yaw_permission_enabled);
 small_yaw_feedback_received=false;mapping();assert(!captured.big_yaw_permission_enabled);
 small_yaw_feedback_received=true;mapping();assert(captured.big_yaw_permission_enabled);
 Remote.left=DR16_Switch_Status_MIDDLE;mapping();assert(!captured.big_yaw_permission_enabled);
 // Sample the waveform, period, mode changes, offline reset and tick wrap.
 Remote.left=DR16_Switch_Status_UP;Remote.right=DR16_Switch_Status_MIDDLE;mapping();
 Remote.right=DR16_Switch_Status_DOWN;tick=100;mapping();near(captured.wz,3.4f);
 tick=1100;mapping();near(captured.wz,2.7f);
 tick=2100;mapping();near(captured.wz,2.0f);
 tick=3100;mapping();near(captured.wz,2.7f);
 tick=4100;mapping();near(captured.wz,3.4f);
 Remote.right=DR16_Switch_Status_MIDDLE;mapping();near(captured.wz,0.0f);
 Remote.right=DR16_Switch_Status_DOWN;tick=4200;mapping();near(captured.wz,3.4f);
 tick=6200;mapping();near(captured.wz,2.0f);
 Remote.left=DR16_Switch_Status_MIDDLE;mapping();near(captured.wz,0.0f);
 Remote.left=DR16_Switch_Status_UP;mapping();near(captured.wz,3.4f);
 tick=8200;mapping();near(captured.wz,2.0f);
 remote_controllable=false;mapping();near(captured.wz,0.0f);
 remote_controllable=true;mapping();near(captured.wz,3.4f);
 Remote.right=DR16_Switch_Status_MIDDLE;mapping();
 tick=UINT32_MAX-499;Remote.right=DR16_Switch_Status_DOWN;mapping();near(captured.wz,3.4f);
 tick=500;mapping();near(captured.wz,2.7f);
 puts("PASS: actual remote mapping + navigation selection, 9 switch combinations, spin/wheel transitions, translation, offline and rearm gates");
}
'''
with tempfile.TemporaryDirectory(prefix='remote_spin_') as directory:
    work = Path(directory)
    (work/'test.cpp').write_text(code)
    subprocess.run(['g++', '-std=c++17', '-Wall', '-Wextra', '-Werror',
                    '-fsanitize=address,undefined', '-I'+str(root/'tests/communication/stubs'), '-I'+str(root/'User_File'),
                    str(work/'test.cpp'), str(root/'User_File/2_Device/Communication/dvc_chassis_link.cpp'),
                    str(root/'User_File/4_Task/tsk_communication.cpp'),
                    str(root/'User_File/2_Device/Communication/dvc_uart_link.cpp'),
                    '-o', str(work/'test')], check=True)
    subprocess.run([str(work/'test')], check=True)
