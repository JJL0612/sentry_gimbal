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
constant = '\n'.join(re.search(r'const float '+name+r' = .*?;', source).group() for name in ('CHASSIS_SPIN_TARGET_OMEGA_RAD_S','BIG_YAW_WHEEL_MAX_RATE_RAD_S'))
code = r'''
#include "2_Device/Communication/dvc_chassis_link.h"
#include <cassert>
#include <cmath>
using std::fabs;
#include <cstdint>
#include <cstdio>
#define REMOTE_PROTOCOL_SBUS 0
enum { DR16_Switch_Status_UP, DR16_Switch_Status_DOWN, DR16_Switch_Status_MIDDLE };
struct RemoteMock {
 int left=DR16_Switch_Status_MIDDLE, right=DR16_Switch_Status_MIDDLE;
 int wheel=0, forward=0, sideways=0;
 int Get_Switch_Left() {return left;} int Get_Switch_Right() {return right;}
 unsigned Get_Valid_Frame_Count() {return 1;}
 int Get_Channel_Centered(int i) {return i==2 ? forward : sideways;}
 int Get_Wheel() {return wheel;}
} Remote;
bool remote_controllable=true, joint_running_enable, motor_running_enable;
unsigned __get_PRIMASK(){return 0;} void __disable_irq(){} void __set_PRIMASK(unsigned){}
chassis_link::Input captured;
chassis_link::Link link;
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
code += constant + '\nvoid mapping(){\n' + body + '\n}\n'
code += r'''
int main(){
 for(int l=0;l<3;l++) for(int r=0;r<3;r++) {
  link.init(123);heartbeat();
  Remote.left=DR16_Switch_Status_DOWN;Remote.right=r;mapping();
  Remote.left=l;Remote.wheel=-660;Remote.forward=330;Remote.sideways=-330;mapping();
  assert(motor_running_enable==(r==DR16_Switch_Status_UP));
  assert(joint_running_enable==(l==DR16_Switch_Status_UP));
  assert(link.enabled()==(l!=DR16_Switch_Status_DOWN));
  near(captured.wz,r==DR16_Switch_Status_DOWN?0.8f:0.0f);
  assert(captured.big_yaw_present);near(captured.big_yaw_rate_rad_s,-1);
  near(captured.vx,-0.15f);near(captured.vy,0.15f);
  Remote.forward=-330;mapping();near(captured.vx,0.15f);near(captured.vy,0.15f);
  Remote.forward=330;
  Remote.wheel=660;mapping();near(captured.big_yaw_rate_rad_s,1);
  near(captured.wz,r==DR16_Switch_Status_DOWN?0.8f:0.0f);
  Remote.right=DR16_Switch_Status_MIDDLE;Remote.wheel=0;mapping();
  assert(!motor_running_enable);near(captured.wz,0);near(captured.big_yaw_rate_rad_s,0);
  Remote.right=DR16_Switch_Status_DOWN;remote_controllable=false;mapping();
  assert(!motor_running_enable&&!joint_running_enable&&!link.enabled());
  near(captured.wz,0);near(captured.vx,0);near(captured.vy,0);near(captured.big_yaw_rate_rad_s,0);
  remote_controllable=true;
 }
 link.init(123);heartbeat();Remote.left=DR16_Switch_Status_MIDDLE;
 Remote.right=DR16_Switch_Status_DOWN;mapping();assert(!link.enabled());
 puts("PASS: actual remote mapping, 9 switch combinations, spin/wheel transitions, translation, offline and rearm gates");
}
'''
with tempfile.TemporaryDirectory(prefix='remote_spin_') as directory:
    work = Path(directory)
    (work/'test.cpp').write_text(code)
    subprocess.run(['g++', '-std=c++17', '-Wall', '-Wextra', '-Werror',
                    '-fsanitize=address,undefined', '-I'+str(root/'User_File'),
                    str(work/'test.cpp'), str(root/'User_File/2_Device/Communication/dvc_chassis_link.cpp'),
                    '-o', str(work/'test')], check=True)
    subprocess.run([str(work/'test')], check=True)
