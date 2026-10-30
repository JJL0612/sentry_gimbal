# 云台导航串口与板间通信

## 接口

| 用途 | 串口 | MCU TX / RX | 配置 |
|---|---|---|---|
| 导航小电脑（板上 USART3 插座） | MCU USART1 / huart1 | PA9 / PA10 | 921600，8N1，普通 UART，无流控 |
| 底盘板间链路 | USART10 / huart10 | PE3 / PE2 | 460800，8N1，v4 |
| VOFA | UART7 / huart7 | PE8 / PE7 | 115200，8N1，JustFloat |

**板上插座名称与 MCU 外设编号不同：用户接线图的“USART3”插座实际引脚为 PA9/PA10，在本工程使用 MCU USART1（huart1）。** PA9 TX 接小电脑 RX，PA10 RX 接小电脑 TX，两端共地，3.3V TTL。使用普通 UART，不使用 RS485。MCU USART3（PD8/PD9）恢复原 1000000 baud RS485 配置，不注册导航接收。

原 USART1 通用链路已停用，不在导航口发送旧通用帧/心跳；其余串口维持原配置。

## 导航速度帧

按用户提供的协议图实现，整帧 **9 字节**；长度字段 `06` 表示六字节速度载荷。

| 偏移 | 字段 | 格式 |
|---|---|---|
| 0 | 帧头 | 固定 AA |
| 1 | 长度 | 固定 06 |
| 2–3 | vx | int16，实际 m/s × 1000 |
| 4–5 | vy | int16，实际 m/s × 1000 |
| 6–7 | wz | int16，实际 rad/s × 1000 |
| 8 | 校验 | 前 8 字节逐字节 XOR |

**图片未注明字节序，本实现约定小端（低字节在前），需要小电脑匹配。** 导航指令语义为 x 向前、y 向左、wz 逆时针。根据实机反馈，原直通映射存在“向前变向左、向左变向后”的 90° 偏差，控制接入处现补偿为 `底盘 vx = 导航 vy`、`底盘 vy = -导航 vx`（平移顺时针旋转 90°），wz 保持原值。解析接口保留原始导航速度，发送接口也不进行坐标变换；补偿仅作用于中档导航控制，遥控方向不变。
例如 vx=1.000、vy=-0.500、wz=0.250：`AA 06 E8 03 0C FE FA 00 4F`。

协议没有 enable、序号、ACK、心跳或里程计字段。每个合法速度帧都更新导航指令；停止时发送三个零速度。建议小电脑每 20–50ms 发送一次。

## 控制模式

- 左拨杆下档：沿用原底盘禁用及重新使能确认流程。
- 左拨杆中档：底盘 vx/vy/wz 使用导航数据；没有有效指令或接收时间超过/等于 200ms，三个速度全部清零，不回退到摇杆。
- 左拨杆上档：使用原遥控速度逻辑，包括右拨杆下档的小陀螺；中档旋转速度由导航 wz 决定。
- 遥控在线、底盘握手、掉线保护和下档重新确认逻辑仍由原板间状态机执行。滚轮的大 Yaw 请求仍沿用原逻辑。
- 最终底盘发送限幅沿用现有值：vx/vy 各 ±0.3m/s，wz ±0.8rad/s。协议表示范围不等于底盘执行范围。

## 实现与调用

`tsk_communication.*` 注册 MCU USART1 DMA IDLE 接收，ISR 复制字节及实际接收时间，`Communication_Task()` 在前台解包。支持半帧、连续多帧、噪声及校验失败后的重同步；半帧间隔超过 100ms 清除。2048 槽环形缓冲实际容量 2047 字节，溢出立即使导航指令失效，前台丢弃积压并等待新帧。当前仅启用 I-Cache；启用 D-Cache 时需处理 DMA 缓存一致性。

`Task1ms_Callback()` 在构造遥控输入后调用 `Navigation_Apply_To_Chassis()` 选择速度来源，再交给现有 `Chassis_Link_Tick()`。`Navigation_Get_Command()` 可在控制中断读取完整快照，超时返回 false 并清零输出；received_ms 为接收回调时间，不会因前台延迟重新延长有效期。enable 和 sequence 只是本地有效标志及接收计数，不在串口帧中。

调试器或前台可调用 `Navigation_Get_Statistics()` 读取合法帧、校验错误和接收溢出计数。

提供 `Navigation_Send_Velocity(vx, vy, wz)` 按相同格式发送三个速度，有限浮点数四舍五入到 0.001，并限幅到 ±32.767；DMA 忙时返回 HAL_BUSY，缓冲区不会被覆盖。仅供前台单一发送者调用。**当前没有自动回传**：图片没有给出回传数据的含义、触发条件及频率，因而不会发送旧协议心跳或伪造里程计。六字段里程计反馈不能套用此帧。

旧通用 `dvc_uart_link.*` 库文件保留，但不再注册 UART1 实例、不发送心跳。该口仅使用图示的 AA 06 导航协议。

## 小电脑封包示例

```python
import struct
from functools import reduce
from operator import xor

def velocity_frame(vx, vy, wz):
    # 三个参数单位为 m/s、m/s、rad/s；小端，int16。
    values = [max(-32767, min(32767, round(v * 1000))) for v in (vx, vy, wz)]
    data = b'\xAA\x06' + struct.pack('<hhh', *values)
    return data + bytes([reduce(xor, data, 0)])

# serial.Serial('/dev/ttyUSB0', 921600, bytesize=8, parity='N', stopbits=1)
# port.write(velocity_frame(0.1, 0.0, 0.0))
```

主机验证见 `tests/communication/README.md`。上板时先架空底盘，确认帧计数和速度方向，再切换中/上档验证模式、停止发包验证 200ms 清零。固件编译和主机测试不能替代实机串口联调。
