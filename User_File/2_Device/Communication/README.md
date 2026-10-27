# DM 板间与导航串口通信框架

> 当前 sentry_chassis 底盘正式通信使用 **USART10：PE3 TX / PE2 RX，460800 8N1**，实现见 `dvc_chassis_link.*` 和 `4_Task/tsk_chassis_link.*`，详情见工程根目录《云台底盘通信与遥控说明.md》。下文 USART1 的通用暂定协议是保留的独立链路，不能用于该底盘。UART7 现用于 VOFA（PE8 TX / PE7 RX，115200 8N1，JustFloat），导航功能默认停用。

默认分配（可在 `4_Task/tsk_communication.cpp` 的 Init 和串口回调中对调）：

| 链路 | MCU 外设 | TX | RX | 参数 |
|---|---|---|---|---|
| 另一块 DM 主控板 | USART1 / huart1 | PA9 | PA10 | 115200，8N1 |
| VOFA | UART7 / huart7 | PE8 | PE7 | 115200，8N1，JustFloat |

TX 接对端 RX，RX 接对端 TX，共地，使用 3.3V TTL。小电脑使用匹配电平的 UART 或 USB-TTL。以上为 MCU 引脚，板上插座位置请按实际板卡丝印/原理图确认。

> 下文导航协议仅为保留的实现说明，当前固件 `COMMUNICATION_NAVIGATION_ENABLE=0`，不会注册UART7导航接收，也不发送导航心跳/反馈。不要在VOFA占用UART7时启用导航。USART1通用链路仍独立运行。

## 代码位置与调用关系

- `dvc_uart_link.*`：通用帧协议、ISR 接收队列、前台拆包、DMA 发送、在线统计。
- `4_Task/tsk_communication.*`：两路实例、业务分发、导航指令缓存、反馈发送、心跳调度。
- `Task_Init()` 调用 `Communication_Init()` 注册 USART1 接收回调；UART7 回调由 VOFA 注册。
- `Task_Loop()` 调用 `Communication_Task()`，解析和业务回调在前台执行。
- `drv_uart.cpp` 对这两路忽略 NORMAL DMA 的 HT 半传输事件，只处理 IDLE/TC。

沿用已生成的 GPIO、DMA、IRQ 配置，未改 CubeMX 外设参数。当前工程仅启用 I-Cache，缓冲区在 DMA 可访问的 RAM_D1；以后启用 D-Cache 时需统一处理 DMA 缓冲区的缓存一致性。

## 暂定协议 v1

这是本框架提供的约定，两端都需要实现；若已有通信协议，可替换设备层解析与封包。

| 字节偏移 | 字段 | 长度 | 说明 |
|---|---|---|---|
| 0 | 帧头 | 2 | A5 5A |
| 2 | 版本 | 1 | 01 |
| 3 | 消息类型 | 1 | 见下表 |
| 4 | payload 长度 | 2 | uint16，小端，0~128 |
| 6 | 帧序号 | 2 | uint16，小端，发送成功后递增，回绕；用于诊断 |
| 8 | payload | N | 业务数据 |
| 8+N | CRC16 | 2 | 小端，覆盖帧头到 payload 最后一个字节 |

CRC-16/CCITT-FALSE：poly=0x1021，init=0xFFFF，refin/refout=false，xorout=0。
序号目前不用于 ACK、去重或重发；业务若需要可靠确认，应扩展消息类型和状态机。

| 类型 | 含义 | payload |
|---|---|---|
| 0x01 | 心跳，两路每 100ms 发送 | 空 |
| 0x10 | DM 板间业务 | 由两块板共同定义，最多 128 字节 |
| 0x20 | 小电脑 → 本板导航速度 | vx、vy、wz 三个 float32 + enable 一个 uint8，合计 13 字节 |
| 0x21 | 本板 → 小电脑导航反馈 | x、y、yaw、vx、vy、wz 六个 float32，合计 24 字节 |

float32 为 IEEE754 小端；不要直接传输 C/C++ 结构体内存（有填充）。速度坐标系为机体 x 向前、y 向左、wz 逆时针为正，单位 m/s、rad/s。反馈位置为双方约定的固定里程计坐标系，单位 m、rad。`enable` 只能为 0 或 1；NaN/Inf 或长度错误的速度包不更新指令。

小电脑封包例子（Python 标准库，无串口库依赖）：

```python
import binascii
import struct

def pack_frame(msg_type, seq, payload=b''):
    if len(payload) > 128:
        raise ValueError('payload too long')
    data = b'\xA5\x5A' + struct.pack('<BBHH', 1, msg_type, len(payload), seq)
    data += payload
    return data + struct.pack('<H', binascii.crc_hqx(data, 0xffff))

# 每 50ms 左右发送新指令；导航停止时发送 enable=0 的零速度。
frame = pack_frame(0x20, 0, struct.pack('<fffB', 0.2, 0.0, 0.0, 1))
# serial_port.write(frame)  # 串口对象由小电脑工程创建，115200/8N1
```

## 后续填写的位置

1. `Board_Frame_Callback()`：定义板间 payload 格式，检查长度并复制到业务缓存。回调 payload 指针只在回调期间有效。
2. `Communication_Task()`：按业务周期发送板间数据，以及真实里程计反馈。当前只自动发送心跳，未用零值伪造里程计。
3. 底盘控制任务：用 `Navigation_Get_Command()` 获取完整快照，然后结合遥控使能、模式和实际底盘限速执行。当前框架缓存导航指令，没有改变已有电机控制。

```cpp
// 前台 Communication_Task 中的板间发送示例：
const uint8_t status[] = {1, 2}; // 示例而非正式协议
if (DM_Board_Send(status, sizeof(status)) == HAL_OK) {
    // 发送已启动；HAL_BUSY 时保持待发数据，下次重试。
}

// 控制任务中的接入示例（需自行接到底盘对象）：
Struct_Navigation_Command cmd;
const bool fresh = Navigation_Get_Command(cmd);
if (fresh && cmd.enable /* && 遥控使能 && 导航模式 */) {
    // Chassis.Set_Target(...经过限速的 cmd.vx_m_s / vy_m_s / wz_rad_s...);
} else {
    // 在导航模式下清零底盘目标速度，不要继续使用上一帧的目标。
}
```

所有 Send/Process 接口仅在前台调用；不要由定时器中断或其他模块直接对 huart1/7 发送，以免覆盖同一 DMA 发送缓冲区。内部复制 payload，调用者的局部数组返回后可释放。每路一次只发送一帧，无 TX 排队；HAL_BUSY 时由业务重试。

`Is_Online()` 默认 500ms 内收到任意 CRC 正确的协议帧才为在线，上电为离线；用于前台链路诊断。导航有效性单独由 `Navigation_Get_Command()` 判断，心跳不会延长速度指令寿命。读取返回 false 时输出结构体清零；返回 true 时仍要检查 enable。两端须持续发心跳/业务帧才能让各自接收状态保持在线。

接收支持半帧、连续多帧、错误 CRC 后重同步；半帧间隔超过 100ms 丢弃。每路队列 2048 字节，溢出计数并丢弃积压数据，等待新的完整帧。可在调试器查看 `Get_Rx_Frame_Count()`、`Get_CRC_Error_Count()`、`Get_Overflow_Count()`。

## 验证

固件使用 CMake + ARM GCC 构建。`tests/communication` 提供主机 HAL 桩测试，验证实际解析器及任务接口；不替代两端串口实机联调。
