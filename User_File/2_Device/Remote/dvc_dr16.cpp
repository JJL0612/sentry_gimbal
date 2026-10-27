/**
 * @file dvc_dr16.cpp
 * @brief 大疆 DR16 遥控器接收机(DBUS)驱动实现
 * @version 1.0
 *
 * @copyright USTC-RoboWalker (c) 2026
 *
 */

/* Includes ------------------------------------------------------------------*/

#include "dvc_dr16.h"

/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function declarations ---------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/

/**
 * @brief 初始化 DR16
 *
 * @note UART 侧由任务层调用 UART_Init(&huart5, UART5_Callback) 完成
 */
void Class_DR16::Init()
{
    memset(&Rx_Data, 0, sizeof(Rx_Data));
    Rx_Data.Switch_Left = DR16_Switch_Status_MIDDLE;
    Rx_Data.Switch_Right = DR16_Switch_Status_MIDDLE;

    Frame_Used = 0;
    Frame_Gap_Cnt_1ms = 0;
    Rx_Frame_Count = 0;
    Pre_Rx_Frame_Count = 0;
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
void Class_DR16::UART_RxCpltCallback(const uint8_t *Rx_Buffer, const uint16_t &Length)
{
    // 距上次接收的间隔过大, 说明上次是残帧: 丢弃已有内容重新同步
    if (Frame_Gap_Cnt_1ms > DR16_FRAME_GAP_TIME_THRESHOLD)
    {
        Frame_Used = 0;
    }
    Frame_Gap_Cnt_1ms = 0;

    for (uint16_t i = 0; i < Length; i++)
    {
        Frame_Buffer[Frame_Used] = Rx_Buffer[i];
        Frame_Used++;

        if (Frame_Used >= DR16_FRAME_LENGTH)
        {
            if (Data_Process() == true)
            {
                Frame_Used = 0;
            }
            else
            {
                // 校验失败: 只丢弃 1 字节使组帧窗口逐字节滑动, 避免整帧错位后长时间无法重新同步
                memmove(Frame_Buffer, Frame_Buffer + 1, DR16_FRAME_LENGTH - 1);
                Frame_Used = DR16_FRAME_LENGTH - 1;
            }
        }
    }
}

/**
 * @brief 1ms 周期回调函数, 维护在线/掉线状态与组帧间隔计时
 *
 */
void Class_DR16::TIM_1ms_PeriodElapsedCallback()
{
    // 有新帧则喂狗
    Update_Flag = (Rx_Frame_Count != Pre_Rx_Frame_Count);
    if (Update_Flag == true)
    {
        Pre_Rx_Frame_Count = Rx_Frame_Count;
        Offline_Cnt_1ms = 0;
        Online = true;
    }
    else if (Offline_Cnt_1ms < DR16_OFFLINE_TIME_THRESHOLD)
    {
        Offline_Cnt_1ms++;
        if (Offline_Cnt_1ms >= DR16_OFFLINE_TIME_THRESHOLD)
        {
            // 连续 DR16_OFFLINE_TIME_THRESHOLD 未收到有效帧: 判为掉线, 数据清零,
            // 避免任务层误用旧值继续控制(使能等安全逻辑统一以 Get_Online() 为准)
            Online = false;
            memset(&Rx_Data, 0, sizeof(Rx_Data));
            Rx_Data.Switch_Left = DR16_Switch_Status_MIDDLE;
            Rx_Data.Switch_Right = DR16_Switch_Status_MIDDLE;
        }
    }

    // 组帧间隔计时(接收回调中清零)
    Frame_Gap_Cnt_1ms++;
}

/**
 * @brief 数据处理过程, 校验并按位域解析 18 字节帧
 *
 * @return bool 本帧是否有效
 */
bool Class_DR16::Data_Process()
{
    uint16_t tmp_channel[DR16_ANALOG_CHANNEL_NUM];
    uint8_t tmp_switch_left;
    uint8_t tmp_switch_right;
    int i;

    // 5 个模拟通道(11bit 位域, 小端)
    tmp_channel[0] = ((uint16_t) Frame_Buffer[0] | ((uint16_t) Frame_Buffer[1] << 8)) & 0x07ff;
    tmp_channel[1] = (((uint16_t) Frame_Buffer[1] >> 3) | ((uint16_t) Frame_Buffer[2] << 5)) & 0x07ff;
    tmp_channel[2] = (((uint16_t) Frame_Buffer[2] >> 6) | ((uint16_t) Frame_Buffer[3] << 2) | ((uint16_t) Frame_Buffer[4] << 10)) & 0x07ff;
    tmp_channel[3] = (((uint16_t) Frame_Buffer[4] >> 1) | ((uint16_t) Frame_Buffer[5] << 7)) & 0x07ff;
    tmp_channel[4] = ((uint16_t) Frame_Buffer[16] | ((uint16_t) Frame_Buffer[17] << 8)) & 0x07ff;

    // DR16 三档拨杆原始编码: 1=上, 2=下, 3=中；只有 0 非法。
    // 左右拨杆必须分别校验，不能把任意一侧的中档(3)误判为整帧无效，
    // 否则一个拨杆拨到中间会导致另一拨杆也停止更新，形成错误的交叉影响。
    tmp_switch_left = (Frame_Buffer[5] >> 6) & 0x03;
    tmp_switch_right = (Frame_Buffer[5] >> 4) & 0x03;
    if (tmp_switch_left == 0 || tmp_switch_right == 0 ||
        Frame_Buffer[12] > 1 || Frame_Buffer[13] > 1)
    {
        Invalid_Frame_Count++;
        return (false);
    }

    // 合法性校验: 任一通道超出量程即判为无效帧(如串口噪声/半帧)
    for (i = 0; i < DR16_ANALOG_CHANNEL_NUM; i++)
    {
        if (tmp_channel[i] < DR16_CHANNEL_MIN || tmp_channel[i] > DR16_CHANNEL_MAX)
        {
            Invalid_Frame_Count++;
            return (false);
        }
    }

    // 校验通过, 更新解析结果
    for (i = 0; i < DR16_ANALOG_CHANNEL_NUM; i++)
    {
        Rx_Data.Analog[i] = Channel_To_Analog(tmp_channel[i]);
    }
    Rx_Data.Switch_Left = (tmp_switch_left == 1) ? (DR16_Switch_Status_UP) : ((tmp_switch_left == 2) ? (DR16_Switch_Status_DOWN) : (DR16_Switch_Status_MIDDLE));
    Rx_Data.Switch_Right = (tmp_switch_right == 1) ? (DR16_Switch_Status_UP) : ((tmp_switch_right == 2) ? (DR16_Switch_Status_DOWN) : (DR16_Switch_Status_MIDDLE));
    Rx_Data.Mouse_X = (int16_t) ((uint16_t) Frame_Buffer[6] | ((uint16_t) Frame_Buffer[7] << 8));
    Rx_Data.Mouse_Y = (int16_t) ((uint16_t) Frame_Buffer[8] | ((uint16_t) Frame_Buffer[9] << 8));
    Rx_Data.Mouse_Z = (int16_t) ((uint16_t) Frame_Buffer[10] | ((uint16_t) Frame_Buffer[11] << 8));
    Rx_Data.Mouse_Left_Pressed = (Frame_Buffer[12] != 0);
    Rx_Data.Mouse_Right_Pressed = (Frame_Buffer[13] != 0);
    Rx_Data.Keyboard_Value = (uint16_t) Frame_Buffer[14] | ((uint16_t) Frame_Buffer[15] << 8);
    Rx_Data.Rx_Time_Stamp = SYS_Timestamp.Get_Current_Timestamp();

    Valid_Frame_Count++;
    Rx_Frame_Count++;

    return (true);
}

/**
 * @brief 原始通道值换算: 扣除中心值并设置中心死区
 *
 * @param __Channel 原始通道值
 * @return int16_t 换算后的通道值
 */
int16_t Class_DR16::Channel_To_Analog(const uint16_t &__Channel) const
{
    int16_t tmp_analog = (int16_t) __Channel - DR16_CHANNEL_CENTER;

    if (tmp_analog <= DR16_CHANNEL_DEAD_ZONE && tmp_analog >= -DR16_CHANNEL_DEAD_ZONE)
    {
        tmp_analog = 0;
    }

    return (tmp_analog);
}

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
