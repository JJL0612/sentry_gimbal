/**
 * @file dvc_sbus.cpp
 * @author 按 SBUS(Futaba S.BUS)协议规范实现, 接口风格对齐 dvc_dr16
 * @brief SBUS 遥控器接收机驱动实现
 * @version 1.0
 * @date 2026-09-29 1.0 新建
 *
 * @copyright USTC-RoboWalker (c) 2026
 *
 */

/* Includes ------------------------------------------------------------------*/

#include "dvc_sbus.h"

/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function declarations ---------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/

/**
 * @brief 初始化 SBUS
 *
 * @note UART 侧由任务层调用 UART_Init(&huart5, UART5_Callback) 完成
 *       (SBUS 的电平反相在 Task_Init() 中配置 UART5 AdvancedInit 的 RXINV)
 */
void Class_SBUS::Init()
{
    memset(&Rx_Data, 0, sizeof(Rx_Data));

    Frame_Used = 0;
    Frame_Gap_Cnt_1ms = 0;
    Rx_Frame_Count = 0;
    Pre_Rx_Frame_Count = 0;
    Byte_Count = 0;
    Offline_Cnt_1ms = 0;
    Online = false;
    Update_Flag = false;
    Valid_Frame_Count = 0;
    Invalid_Frame_Count = 0;
}

/**
 * @brief UART接收回调函数, 逐字节组帧
 *
 * @param Rx_Buffer 接收缓冲区
 * @param Length 接收数据长度
 */
void Class_SBUS::UART_RxCpltCallback(const uint8_t *Rx_Buffer, const uint16_t &Length)
{
    // 距上次接收的间隔过大, 说明上次是残帧: 丢弃已有内容重新同步
    if (Frame_Gap_Cnt_1ms > SBUS_FRAME_GAP_TIME_THRESHOLD)
    {
        Frame_Used = 0;
    }
    Frame_Gap_Cnt_1ms = 0;
    Byte_Count += Length;

    for (uint16_t i = 0; i < Length; i++)
    {
        // 未开始组帧时先找帧头, 减少错位概率
        if (Frame_Used == 0 && Rx_Buffer[i] != SBUS_FRAME_HEAD)
        {
            continue;
        }

        Frame_Buffer[Frame_Used] = Rx_Buffer[i];
        Frame_Used++;

        if (Frame_Used >= SBUS_FRAME_LENGTH)
        {
            if (Data_Process() == true)
            {
                Frame_Used = 0;
            }
            else
            {
                // 校验失败: 只丢弃 1 字节使组帧窗口逐字节滑动, 避免整帧错位后长时间无法重新同步
                memmove(Frame_Buffer, Frame_Buffer + 1, SBUS_FRAME_LENGTH - 1);
                Frame_Used = SBUS_FRAME_LENGTH - 1;
            }
        }
    }
}

/**
 * @brief 1ms 周期回调函数, 维护在线/掉线状态与组帧间隔计时
 *
 */
void Class_SBUS::TIM_1ms_PeriodElapsedCallback()
{
    // 有新帧则喂狗
    Update_Flag = (Rx_Frame_Count != Pre_Rx_Frame_Count);
    if (Update_Flag == true)
    {
        Pre_Rx_Frame_Count = Rx_Frame_Count;
        Offline_Cnt_1ms = 0;
        Online = true;
    }
    else if (Offline_Cnt_1ms < SBUS_OFFLINE_TIME_THRESHOLD)
    {
        Offline_Cnt_1ms++;
        if (Offline_Cnt_1ms >= SBUS_OFFLINE_TIME_THRESHOLD)
        {
            // 连续 SBUS_OFFLINE_TIME_THRESHOLD 未收到有效帧: 判为掉线, 数据清零,
            // 避免任务层误用旧值继续控制(使能等安全逻辑统一以 Get_Online() 为准)
            Online = false;
            memset(&Rx_Data, 0, sizeof(Rx_Data));
        }
    }

    // 组帧间隔计时(接收回调中清零)
    Frame_Gap_Cnt_1ms++;
}

/**
 * @brief 数据处理过程, 校验并按位域解析 25 字节帧
 *
 * @return bool 本帧是否有效
 */
bool Class_SBUS::Data_Process()
{
    uint16_t tmp_channel[SBUS_CHANNEL_NUM];
    uint8_t tmp_flags;
    int i;

    // 帧头/帧尾/标志位高位校验(正确对齐的帧才会全部通过, 同时用于重同步)
    tmp_flags = Frame_Buffer[23];
    if (Frame_Buffer[0] != SBUS_FRAME_HEAD || (tmp_flags & 0xf0) != 0x00 ||
        !(Frame_Buffer[24] == 0x00 || (Frame_Buffer[24] & 0x0f) == 0x04))
    {
        Invalid_Frame_Count++;
        return (false);
    }

    // 16 个通道(11bit 小端位域, 从 Frame_Buffer[1] 开始)
    tmp_channel[0] = ((uint16_t) Frame_Buffer[1] | ((uint16_t) Frame_Buffer[2] << 8)) & 0x07ff;
    tmp_channel[1] = (((uint16_t) Frame_Buffer[2] >> 3) | ((uint16_t) Frame_Buffer[3] << 5)) & 0x07ff;
    tmp_channel[2] = (((uint16_t) Frame_Buffer[3] >> 6) | ((uint16_t) Frame_Buffer[4] << 2) | ((uint16_t) Frame_Buffer[5] << 10)) & 0x07ff;
    tmp_channel[3] = (((uint16_t) Frame_Buffer[5] >> 1) | ((uint16_t) Frame_Buffer[6] << 7)) & 0x07ff;
    tmp_channel[4] = (((uint16_t) Frame_Buffer[6] >> 4) | ((uint16_t) Frame_Buffer[7] << 4)) & 0x07ff;
    tmp_channel[5] = (((uint16_t) Frame_Buffer[7] >> 7) | ((uint16_t) Frame_Buffer[8] << 1) | ((uint16_t) Frame_Buffer[9] << 9)) & 0x07ff;
    tmp_channel[6] = (((uint16_t) Frame_Buffer[9] >> 2) | ((uint16_t) Frame_Buffer[10] << 6)) & 0x07ff;
    tmp_channel[7] = (((uint16_t) Frame_Buffer[10] >> 5) | ((uint16_t) Frame_Buffer[11] << 3)) & 0x07ff;
    tmp_channel[8] = ((uint16_t) Frame_Buffer[12] | ((uint16_t) Frame_Buffer[13] << 8)) & 0x07ff;
    tmp_channel[9] = (((uint16_t) Frame_Buffer[13] >> 3) | ((uint16_t) Frame_Buffer[14] << 5)) & 0x07ff;
    tmp_channel[10] = (((uint16_t) Frame_Buffer[14] >> 6) | ((uint16_t) Frame_Buffer[15] << 2) | ((uint16_t) Frame_Buffer[16] << 10)) & 0x07ff;
    tmp_channel[11] = (((uint16_t) Frame_Buffer[16] >> 1) | ((uint16_t) Frame_Buffer[17] << 7)) & 0x07ff;
    tmp_channel[12] = (((uint16_t) Frame_Buffer[17] >> 4) | ((uint16_t) Frame_Buffer[18] << 4)) & 0x07ff;
    tmp_channel[13] = (((uint16_t) Frame_Buffer[18] >> 7) | ((uint16_t) Frame_Buffer[19] << 1) | ((uint16_t) Frame_Buffer[20] << 9)) & 0x07ff;
    tmp_channel[14] = (((uint16_t) Frame_Buffer[20] >> 2) | ((uint16_t) Frame_Buffer[21] << 6)) & 0x07ff;
    tmp_channel[15] = (((uint16_t) Frame_Buffer[21] >> 5) | ((uint16_t) Frame_Buffer[22] << 3)) & 0x07ff;

    // 通道量程软校验(规范量程 172~1811, 此处放宽到 100~1970): 剔除半帧/串口噪声
    for (i = 0; i < SBUS_CHANNEL_NUM; i++)
    {
        if (tmp_channel[i] < SBUS_CHANNEL_VALID_MIN || tmp_channel[i] > SBUS_CHANNEL_VALID_MAX)
        {
            Invalid_Frame_Count++;
            return (false);
        }
    }

    // 校验通过, 更新解析结果
    for (i = 0; i < SBUS_CHANNEL_NUM; i++)
    {
        Rx_Data.Channel[i] = tmp_channel[i];
    }
    Rx_Data.Digital_Channel_17 = ((tmp_flags & 0x01) != 0);
    Rx_Data.Digital_Channel_18 = ((tmp_flags & 0x02) != 0);
    Rx_Data.Frame_Lost = ((tmp_flags & 0x04) != 0);
    Rx_Data.Fail_Safe = ((tmp_flags & 0x08) != 0);
    Rx_Data.Rx_Time_Stamp = SYS_Timestamp.Get_Current_Timestamp();

    Valid_Frame_Count++;
    Rx_Frame_Count++;

    return (true);
}

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
