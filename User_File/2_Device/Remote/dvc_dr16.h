/**
 * @file dvc_dr16.h
 * @brief 大疆 DR16 遥控器接收机(DBUS)驱动: 100kBaud / 9bit(8数据+偶校验) / 18 字节定长帧
 * @version 1.0
 *
 * @note 接线: 接收机 TX -> 主控 UART5_RX(PD2), 接收机 4.5~5.5V 供电, 与主控共地。
 *       CubeMX 已按 DR16 配好: MX_UART5_Init() = 100000 baud / 9bit(8数据+偶校验) / 2 stop / 仅 RX,
 *       因此任务层只需 UART_Init(&huart5, UART5_Callback) 即可(见 tsk_config_and_callback.cpp)。
 *       若换成 SBUS(Futaba)接收机: 100kBaud / 8E2 / 逻辑反相 / 25 字节帧, 帧格式与本驱动不同, 不能直接用。
 *
 * @note 帧结构(18 字节):
 *       [0..1]右摇杆X [1..2]右摇杆Y [2..4]左摇杆Y [4..5]左摇杆X 按 11bit 位域拼接,
 *       [5]高位为左右拨杆, [6..13]鼠标(含左右键), [14..15]键盘位图, [16..17]滚轮。
 *
 * @copyright USTC-RoboWalker (c) 2026
 *
 */

#ifndef DVC_DR16_H
#define DVC_DR16_H

/* Includes ------------------------------------------------------------------*/

#include "1_Middleware/Driver/UART/drv_uart.h"
#include "1_Middleware/System/Timestamp/sys_timestamp.h"
#include <stdint.h>
#include <string.h>

/* Exported macros -----------------------------------------------------------*/

// 单帧字节数(DR16/DBUS 定长帧)
#define DR16_FRAME_LENGTH (18)
// 模拟通道数量: 右摇杆X/Y, 左摇杆Y/X, 滚轮
#define DR16_ANALOG_CHANNEL_NUM (5)
// 掉线判定时间, ms: 连续该时长未收到有效帧即判为掉线
#define DR16_OFFLINE_TIME_THRESHOLD (100)
// 组帧字节间隔阈值, ms: 相邻两次接收间隔超过该值即认为上次是残帧, 丢弃后重新同步
#define DR16_FRAME_GAP_TIME_THRESHOLD (5)
// 通道原始刻度: 中心值/下限/上限/中心死区
#define DR16_CHANNEL_CENTER (1024)
#define DR16_CHANNEL_MIN (364)
#define DR16_CHANNEL_MAX (1684)
#define DR16_CHANNEL_DEAD_ZONE (10)

/* Exported types ------------------------------------------------------------*/

/**
 * @brief DR16 拨杆状态
 *
 */
enum Enum_DR16_Switch_Status
{
    DR16_Switch_Status_UP = 0,
    DR16_Switch_Status_DOWN,
    DR16_Switch_Status_MIDDLE,
};

/**
 * @brief DR16 解析后的遥控器数据
 *
 */
struct Struct_DR16_Rx_Data
{
    // 5 个模拟通道, 已扣除中心 DR16_CHANNEL_CENTER 并内置中心死区, 量程约 ±660
    // 顺序与接收机一致: [0]右摇杆X [1]右摇杆Y [2]左摇杆Y [3]左摇杆X [4]滚轮
    int16_t Analog[DR16_ANALOG_CHANNEL_NUM];
    // 左/右拨杆
    Enum_DR16_Switch_Status Switch_Left;
    Enum_DR16_Switch_Status Switch_Right;
    // 鼠标
    int16_t Mouse_X;
    int16_t Mouse_Y;
    int16_t Mouse_Z;
    bool Mouse_Left_Pressed;
    bool Mouse_Right_Pressed;
    // 键盘按键位图
    uint16_t Keyboard_Value;
    // 本帧接收时刻的时间戳
    uint64_t Rx_Time_Stamp;
};

/**
 * @brief Reusable, 大疆 DR16 遥控器接收机(DBUS)
 *
 */
class Class_DR16
{
public:
    // 解析后的遥控器数据
    Struct_DR16_Rx_Data Rx_Data;

    void Init();

    inline bool Get_Online() const;

    inline bool Get_Update_Flag() const;

    inline int16_t Get_Right_X() const;

    inline int16_t Get_Right_Y() const;

    inline int16_t Get_Left_Y() const;

    inline int16_t Get_Left_X() const;

    inline int16_t Get_Wheel() const;

    inline Enum_DR16_Switch_Status Get_Switch_Left() const;

    inline Enum_DR16_Switch_Status Get_Switch_Right() const;

    inline uint32_t Get_Valid_Frame_Count() const;

    inline uint32_t Get_Invalid_Frame_Count() const;

    // 以下三项为与 SBUS 统一接口而提供(DR16/DBUS 帧内无失控保护位, 恒为 false)
    inline bool Get_Fail_Safe() const;

    inline bool Get_Frame_Lost() const;

    // 与 SBUS 统一的通道取值接口: Get_Channel 返回原始刻度, Get_Channel_Centered 返回扣中心后的值
    inline uint16_t Get_Channel(const uint8_t &__Index) const;

    inline int16_t Get_Channel_Centered(const uint8_t &__Index) const;

    void UART_RxCpltCallback(const uint8_t *Rx_Buffer, const uint16_t &Length);

    void TIM_1ms_PeriodElapsedCallback();

protected:
    // 内部变量

    // 组帧缓存与已用长度
    uint8_t Frame_Buffer[DR16_FRAME_LENGTH];
    uint8_t Frame_Used = 0;
    // 组帧字节间隔计时, ms
    uint32_t Frame_Gap_Cnt_1ms = 0;
    // 有效帧累计计数(接收中断中自增)与上一个 1ms 周期的计数
    uint32_t Rx_Frame_Count = 0;
    uint32_t Pre_Rx_Frame_Count = 0;
    // 连续无有效帧累计时长, ms
    uint32_t Offline_Cnt_1ms = 0;

    // 读变量

    // 在线标志(1ms 周期判定, 由任务层的安全链路使用)
    bool Online = false;
    // 本 1ms 周期是否收到过新帧
    bool Update_Flag = false;
    // 有效/无效帧统计
    uint32_t Valid_Frame_Count = 0;
    uint32_t Invalid_Frame_Count = 0;

    // 内部函数

    bool Data_Process();

    int16_t Channel_To_Analog(const uint16_t &__Channel) const;
};

/* Exported variables --------------------------------------------------------*/

/* Exported function declarations --------------------------------------------*/

/**
 * @brief 获取遥控器是否在线(连续 DR16_OFFLINE_TIME_THRESHOLD 未收到有效帧即为离线)
 *
 * @return bool 是否在线
 */
inline bool Class_DR16::Get_Online() const
{
    return (Online);
}

/**
 * @brief 获取本 1ms 周期是否收到过新帧
 *
 * @return bool 是否收到新帧
 */
inline bool Class_DR16::Get_Update_Flag() const
{
    return (Update_Flag);
}

/**
 * @brief 获取右摇杆X(原始刻度扣除中心, 右为正/左为负)
 *
 * @return int16_t 右摇杆X
 */
inline int16_t Class_DR16::Get_Right_X() const
{
    return (Rx_Data.Analog[0]);
}

/**
 * @brief 获取右摇杆Y(原始刻度扣除中心)
 *
 * @return int16_t 右摇杆Y
 */
inline int16_t Class_DR16::Get_Right_Y() const
{
    return (Rx_Data.Analog[1]);
}

/**
 * @brief 获取左摇杆Y(原始刻度扣除中心)
 *
 * @return int16_t 左摇杆Y
 */
inline int16_t Class_DR16::Get_Left_Y() const
{
    return (Rx_Data.Analog[2]);
}

/**
 * @brief 获取左摇杆X(原始刻度扣除中心, 右为正/左为负)
 *
 * @return int16_t 左摇杆X
 */
inline int16_t Class_DR16::Get_Left_X() const
{
    return (Rx_Data.Analog[3]);
}

/**
 * @brief 获取滚轮(原始刻度扣除中心)
 *
 * @return int16_t 滚轮
 */
inline int16_t Class_DR16::Get_Wheel() const
{
    return (Rx_Data.Analog[4]);
}

/**
 * @brief 获取左拨杆状态
 *
 * @return Enum_DR16_Switch_Status 左拨杆状态
 */
inline Enum_DR16_Switch_Status Class_DR16::Get_Switch_Left() const
{
    return (Rx_Data.Switch_Left);
}

/**
 * @brief 获取右拨杆状态
 *
 * @return Enum_DR16_Switch_Status 右拨杆状态
 */
inline Enum_DR16_Switch_Status Class_DR16::Get_Switch_Right() const
{
    return (Rx_Data.Switch_Right);
}

/**
 * @brief 获取累计有效帧数
 *
 * @return uint32_t 累计有效帧数
 */
inline uint32_t Class_DR16::Get_Valid_Frame_Count() const
{
    return (Valid_Frame_Count);
}

/**
 * @brief 获取累计无效帧数
 *
 * @return uint32_t 累计无效帧数
 */
inline uint32_t Class_DR16::Get_Invalid_Frame_Count() const
{
    return (Invalid_Frame_Count);
}

/**
 * @brief 获取失控保护标志(DR16/DBUS 帧内无该位, 恒为 false; 与 SBUS 统一接口)
 *
 * @return bool 恒为 false
 */
inline bool Class_DR16::Get_Fail_Safe() const
{
    return (false);
}

/**
 * @brief 获取帧丢失标志(DR16/DBUS 帧内无该位, 恒为 false; 与 SBUS 统一接口)
 *
 * @return bool 恒为 false
 */
inline bool Class_DR16::Get_Frame_Lost() const
{
    return (false);
}

/**
 * @brief 获取指定通道的原始刻度值(仅 0~4 通道有效, 越界返回 0)
 *
 * @param __Index 通道序号
 * @return uint16_t 原始刻度值
 */
inline uint16_t Class_DR16::Get_Channel(const uint8_t &__Index) const
{
    if (__Index >= DR16_ANALOG_CHANNEL_NUM)
    {
        return (0);
    }

    return ((uint16_t) (Rx_Data.Analog[__Index] + DR16_CHANNEL_CENTER));
}

/**
 * @brief 获取指定通道扣除中心并带死区的值(仅 0~4 通道有效, 越界返回 0)
 *
 * @param __Index 通道序号
 * @return int16_t 扣中心后的通道值
 */
inline int16_t Class_DR16::Get_Channel_Centered(const uint8_t &__Index) const
{
    if (__Index >= DR16_ANALOG_CHANNEL_NUM)
    {
        return (0);
    }

    return (Rx_Data.Analog[__Index]);
}

#endif

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
