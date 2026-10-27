/**
 * @file dvc_sbus.h
 * @author 按 SBUS(Futaba S.BUS)协议规范实现, 接口风格对齐 dvc_dr16
 * @brief SBUS 遥控器接收机驱动: 100kBaud / 8E2 / 25 字节定长帧 / 16 通道 11bit
 * @version 1.0
 * @date 2026-09-29 1.0 新建(本工程接收机为 SBUS 输出, 与 DR16/DBUS 帧格式不同)
 *
 * @note 接线: 接收机 SBUS 输出 -> 主控 UART5_RX(PD2), 接收机供电(4.5~5.5V)并与主控共地。
 * @note 电平反相: SBUS 是**逻辑反相**串口(空闲为高电平)。本工程用 UART 内部反相实现:
 *       任务层 Task_Init() 中按 REMOTE_SBUS_RX_INVERT 配置 huart5.AdvancedInit 的 RXINV 后重新 HAL_UART_Init。
 *       若板卡 SBUS 通路已自带反相器, 则把该宏置 0。
 * @note 串口格式与当前 CubeMX 配置一致: 100000 baud / 9bit(=8 数据 + 偶校验) / 2 stop / 仅 RX,
 *       解析层实现: 25 字节帧 [0]=0x0F, [1..22]=16 通道 11bit 小端位域, [23]=标志位, [24]=帧尾。
 *
 * @copyright USTC-RoboWalker (c) 2026
 *
 */

#ifndef DVC_SBUS_H
#define DVC_SBUS_H

/* Includes ------------------------------------------------------------------*/

#include "1_Middleware/Driver/UART/drv_uart.h"
#include "1_Middleware/System/Timestamp/sys_timestamp.h"
#include <stdint.h>
#include <string.h>

/* Exported macros -----------------------------------------------------------*/

// 单帧字节数(SBUS 定长帧)
#define SBUS_FRAME_LENGTH (25)
// SBUS 通道数量
#define SBUS_CHANNEL_NUM (16)
// 帧起始字节
#define SBUS_FRAME_HEAD (0x0F)
// 掉线判定时间, ms: 连续该时长未收到有效帧即判为掉线(标准帧周期约 14ms)
#define SBUS_OFFLINE_TIME_THRESHOLD (100)
// 组帧字节间隔阈值, ms: 相邻两次接收间隔超过该值即认为上次是残帧, 丢弃后重新同步
#define SBUS_FRAME_GAP_TIME_THRESHOLD (5)
// 通道中心值/中心死区/合法范围(Futaba 规范: 172=-100%, 992=中心, 1811=+100%)
#define SBUS_CHANNEL_CENTER (992)
#define SBUS_CHANNEL_DEAD_ZONE (10)
#define SBUS_CHANNEL_VALID_MIN (100)
#define SBUS_CHANNEL_VALID_MAX (1970)

/* Exported types ------------------------------------------------------------*/

/**
 * @brief SBUS 解析后的接收机数据
 *
 */
struct Struct_SBUS_Rx_Data
{
    // 16 个通道的原始 11bit 值(规范量程 172~1811)
    // 通道与发射机功能的对应关系由收发机决定(常见 AETR: ch0=副翼/横滚, ch1=升降/俯仰, ch2=油门, ch3=方向),
    // 具体哪个通道对应哪个摇杆请用 Get_Channel_Centered() 实测确认
    uint16_t Channel[SBUS_CHANNEL_NUM];
    // 数字通道 17/18(两位拨杆等)
    bool Digital_Channel_17;
    bool Digital_Channel_18;
    // 帧丢失标志
    bool Frame_Lost;
    // 失控保护标志(发射机/接收机进入 failsafe)
    bool Fail_Safe;
    // 本帧接收时刻的时间戳
    uint64_t Rx_Time_Stamp;
};

/**
 * @brief Reusable, SBUS 遥控器接收机
 *
 */
class Class_SBUS
{
public:
    // 解析后的接收机数据
    Struct_SBUS_Rx_Data Rx_Data;

    void Init();

    inline bool Get_Online() const;

    inline bool Get_Update_Flag() const;

    inline uint16_t Get_Channel(const uint8_t &__Index) const;

    inline int16_t Get_Channel_Centered(const uint8_t &__Index) const;

    inline bool Get_Digital_Channel_17() const;

    inline bool Get_Digital_Channel_18() const;

    inline bool Get_Frame_Lost() const;

    inline bool Get_Fail_Safe() const;

    inline uint32_t Get_Valid_Frame_Count() const;

    inline uint32_t Get_Invalid_Frame_Count() const;

    inline uint32_t Get_Byte_Count() const;

    void UART_RxCpltCallback(const uint8_t *Rx_Buffer, const uint16_t &Length);

    void TIM_1ms_PeriodElapsedCallback();

protected:
    // 内部变量

    // 组帧缓存与已用长度
    uint8_t Frame_Buffer[SBUS_FRAME_LENGTH];
    uint8_t Frame_Used = 0;
    // 组帧字节间隔计时, ms
    uint32_t Frame_Gap_Cnt_1ms = 0;
    // 有效帧累计计数(接收中断中自增)与上一个 1ms 周期的计数
    uint32_t Rx_Frame_Count = 0;
    uint32_t Pre_Rx_Frame_Count = 0;
    // 累计收到的字节数(用于区分"完全没数据"与"有数据但帧不合法/反相没配对")
    uint32_t Byte_Count = 0;
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
};

/* Exported variables --------------------------------------------------------*/

/* Exported function declarations --------------------------------------------*/

/**
 * @brief 获取接收机是否在线(连续 SBUS_OFFLINE_TIME_THRESHOLD 未收到有效帧即为离线)
 *
 * @return bool 是否在线
 */
inline bool Class_SBUS::Get_Online() const
{
    return (Online);
}

/**
 * @brief 获取本 1ms 周期是否收到过新帧
 *
 * @return bool 是否收到新帧
 */
inline bool Class_SBUS::Get_Update_Flag() const
{
    return (Update_Flag);
}

/**
 * @brief 获取指定通道的原始 11bit 值
 *
 * @param __Index 通道序号 0~15
 * @return uint16_t 原始值(序号越界时返回 0)
 */
inline uint16_t Class_SBUS::Get_Channel(const uint8_t &__Index) const
{
    if (__Index >= SBUS_CHANNEL_NUM)
    {
        return (0);
    }

    return (Rx_Data.Channel[__Index]);
}

/**
 * @brief 获取指定通道扣除中心(992)并带中心死区的值, 中心为 0, 约 ±819
 *
 * @param __Index 通道序号 0~15
 * @return int16_t 扣中心后的通道值(序号越界时返回 0)
 */
inline int16_t Class_SBUS::Get_Channel_Centered(const uint8_t &__Index) const
{
    if (__Index >= SBUS_CHANNEL_NUM)
    {
        return (0);
    }

    int16_t tmp_channel = (int16_t) Rx_Data.Channel[__Index] - SBUS_CHANNEL_CENTER;

    if (tmp_channel <= SBUS_CHANNEL_DEAD_ZONE && tmp_channel >= -SBUS_CHANNEL_DEAD_ZONE)
    {
        tmp_channel = 0;
    }

    return (tmp_channel);
}

/**
 * @brief 获取数字通道 17
 *
 * @return bool 数字通道 17
 */
inline bool Class_SBUS::Get_Digital_Channel_17() const
{
    return (Rx_Data.Digital_Channel_17);
}

/**
 * @brief 获取数字通道 18
 *
 * @return bool 数字通道 18
 */
inline bool Class_SBUS::Get_Digital_Channel_18() const
{
    return (Rx_Data.Digital_Channel_18);
}

/**
 * @brief 获取帧丢失标志
 *
 * @return bool 帧丢失
 */
inline bool Class_SBUS::Get_Frame_Lost() const
{
    return (Rx_Data.Frame_Lost);
}

/**
 * @brief 获取失控保护标志(与 DR16 对齐的接口, 供任务层安全链路统一判断)
 *
 * @return bool 是否进入失控保护
 */
inline bool Class_SBUS::Get_Fail_Safe() const
{
    return (Rx_Data.Fail_Safe);
}

/**
 * @brief 获取累计有效帧数
 *
 * @return uint32_t 累计有效帧数
 */
inline uint32_t Class_SBUS::Get_Valid_Frame_Count() const
{
    return (Valid_Frame_Count);
}

/**
 * @brief 获取累计无效帧数
 *
 * @return uint32_t 累计无效帧数
 */
inline uint32_t Class_SBUS::Get_Invalid_Frame_Count() const
{
    return (Invalid_Frame_Count);
}

/**
 * @brief 获取累计收到的字节数(用于区分"没数据"与"有数据但帧不合法/反相没配对")
 *
 * @return uint32_t 累计字节数
 */
inline uint32_t Class_SBUS::Get_Byte_Count() const
{
    return (Byte_Count);
}

#endif

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
