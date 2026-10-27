/**
 * @file dvc_motor_dm.cpp
 * @author yssickjgd (1345578933@qq.com)
 * @brief 达妙电机配置与操作
 * @version 0.1
 * @date 2023-08-30 0.1 新增
 *
 * @copyright USTC-RoboWalker (c) 2023
 *
 */

/* Includes ------------------------------------------------------------------*/

#include "dvc_motor_dm.h"

/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

// 清除电机错误信息, 传统模式有效
uint8_t DM_Motor_CAN_Message_Clear_Error[8] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfb};
// 使能电机, 传统模式有效
uint8_t DM_Motor_CAN_Message_Enter[8] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfc};
// 失能电机, 传统模式有效
uint8_t DM_Motor_CAN_Message_Exit[8] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfd};
// 保存当前电机位置为零点, 传统模式有效
uint8_t DM_Motor_CAN_Message_Save_Zero[8] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfe};

/* Private function declarations ---------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/

/**
 * @brief 分配CAN发送缓冲区, 一拖四模式有效
 *
 * @param hcan CAN编号
 * @param __CAN_ID CAN ID
 * @param __DJI_Motor_Driver_Version 大疆驱动版本, 当且仅当当前被分配电机为6020, 且是电流驱动新版本时选2023, 否则都是default
 * @return uint8_t* 缓冲区指针
 */
uint8_t *allocate_tx_data(const FDCAN_HandleTypeDef *hcan, const Enum_Motor_DM_Motor_ID_1_To_4 &__CAN_Rx_ID_1_To_4)
{
    uint8_t *tmp_tx_data_ptr;
    if (hcan == &hfdcan1)
    {
        switch (__CAN_Rx_ID_1_To_4)
        {
        case (Motor_DM_ID_0x301):
        {
            tmp_tx_data_ptr = &(CAN1_0x3fe_Tx_Data[0]);

            break;
        }
        case (Motor_DM_ID_0x302):
        {
            tmp_tx_data_ptr = &(CAN1_0x3fe_Tx_Data[2]);

            break;
        }
        case (Motor_DM_ID_0x303):
        {
            tmp_tx_data_ptr = &(CAN1_0x3fe_Tx_Data[4]);

            break;
        }
        case (Motor_DM_ID_0x304):
        {
            tmp_tx_data_ptr = &(CAN1_0x3fe_Tx_Data[6]);

            break;
        }
        case (Motor_DM_ID_0x305):
        {
            tmp_tx_data_ptr = &(CAN1_0x4fe_Tx_Data[0]);

            break;
        }
        case (Motor_DM_ID_0x306):
        {
            tmp_tx_data_ptr = &(CAN1_0x4fe_Tx_Data[2]);

            break;
        }
        case (Motor_DM_ID_0x307):
        {
            tmp_tx_data_ptr = &(CAN1_0x4fe_Tx_Data[4]);

            break;
        }
        case (Motor_DM_ID_0x308):
        {
            tmp_tx_data_ptr = &(CAN1_0x4fe_Tx_Data[6]);

            break;
        }
        }
    }
    else if (hcan == &hfdcan2)
    {
        switch (__CAN_Rx_ID_1_To_4)
        {
        case (Motor_DM_ID_0x301):
        {
            tmp_tx_data_ptr = &(CAN2_0x3fe_Tx_Data[0]);

            break;
        }
        case (Motor_DM_ID_0x302):
        {
            tmp_tx_data_ptr = &(CAN2_0x3fe_Tx_Data[2]);

            break;
        }
        case (Motor_DM_ID_0x303):
        {
            tmp_tx_data_ptr = &(CAN2_0x3fe_Tx_Data[4]);

            break;
        }
        case (Motor_DM_ID_0x304):
        {
            tmp_tx_data_ptr = &(CAN2_0x3fe_Tx_Data[6]);

            break;
        }
        case (Motor_DM_ID_0x305):
        {
            tmp_tx_data_ptr = &(CAN2_0x4fe_Tx_Data[0]);

            break;
        }
        case (Motor_DM_ID_0x306):
        {
            tmp_tx_data_ptr = &(CAN2_0x4fe_Tx_Data[2]);

            break;
        }
        case (Motor_DM_ID_0x307):
        {
            tmp_tx_data_ptr = &(CAN2_0x4fe_Tx_Data[4]);

            break;
        }
        case (Motor_DM_ID_0x308):
        {
            tmp_tx_data_ptr = &(CAN2_0x4fe_Tx_Data[6]);

            break;
        }
        }
    }
    else if (hcan == &hfdcan3)
    {
        switch (__CAN_Rx_ID_1_To_4)
        {
        case (Motor_DM_ID_0x301):
        {
            tmp_tx_data_ptr = &(CAN3_0x3fe_Tx_Data[0]);

            break;
        }
        case (Motor_DM_ID_0x302):
        {
            tmp_tx_data_ptr = &(CAN3_0x3fe_Tx_Data[2]);

            break;
        }
        case (Motor_DM_ID_0x303):
        {
            tmp_tx_data_ptr = &(CAN3_0x3fe_Tx_Data[4]);

            break;
        }
        case (Motor_DM_ID_0x304):
        {
            tmp_tx_data_ptr = &(CAN3_0x3fe_Tx_Data[6]);

            break;
        }
        case (Motor_DM_ID_0x305):
        {
            tmp_tx_data_ptr = &(CAN3_0x4fe_Tx_Data[0]);

            break;
        }
        case (Motor_DM_ID_0x306):
        {
            tmp_tx_data_ptr = &(CAN3_0x4fe_Tx_Data[2]);

            break;
        }
        case (Motor_DM_ID_0x307):
        {
            tmp_tx_data_ptr = &(CAN3_0x4fe_Tx_Data[4]);

            break;
        }
        case (Motor_DM_ID_0x308):
        {
            tmp_tx_data_ptr = &(CAN3_0x4fe_Tx_Data[6]);

            break;
        }
        }
    }
    else
    {
        tmp_tx_data_ptr = nullptr;
    }
    return (tmp_tx_data_ptr);
}

/**
 * @brief 电机初始化
 *
 * @param hcan 绑定的CAN总线
 * @param __CAN_Rx_ID 收数据绑定的CAN ID, 与上位机驱动参数Master_ID保持一致, 传统模式有效
 * @param __CAN_Tx_ID 发数据绑定的CAN ID, 是上位机驱动参数CAN_ID加上控制模式的偏移量, 传统模式有效
 * @param __Motor_DM_Control_Method 电机控制方式
 * @param __Angle_Max 最大位置, 与上位机控制幅值PMAX保持一致, 传统模式有效
 * @param __Omega_Max 最大速度, 与上位机控制幅值VMAX保持一致, 传统模式有效
 * @param __Torque_Max 最大扭矩, 与上位机控制幅值TMAX保持一致, 传统模式有效
 * @param __Current_Max 最大电流, 与上位机串口中上电打印电流保持一致, EMIT模式需要
 */
void Class_Motor_DM_Normal::Init(const FDCAN_HandleTypeDef *hcan, const uint8_t &__CAN_Rx_ID, const uint8_t &__CAN_Tx_ID, const Enum_Motor_DM_Control_Method &__Motor_DM_Control_Method, const float &__Angle_Max, const float &__Omega_Max, const float &__Torque_Max, const float &__Current_Max)
{
    if (hcan->Instance == FDCAN1)
    {
        CAN_Manage_Object = &CAN1_Manage_Object;
    }
    else if (hcan->Instance == FDCAN2)
    {
        CAN_Manage_Object = &CAN2_Manage_Object;
    }
    else if (hcan->Instance == FDCAN3)
    {
        CAN_Manage_Object = &CAN3_Manage_Object;
    }
    CAN_Rx_ID = __CAN_Rx_ID;
    switch (__Motor_DM_Control_Method)
    {
    case (Motor_DM_Control_Method_NORMAL_MIT):
    {
        CAN_Tx_ID = __CAN_Tx_ID;
        break;
    }
    case (Motor_DM_Control_Method_NORMAL_ANGLE_OMEGA):
    {
        CAN_Tx_ID = __CAN_Tx_ID + 0x100;
        break;
    }
    case (Motor_DM_Control_Method_NORMAL_OMEGA):
    {
        CAN_Tx_ID = __CAN_Tx_ID + 0x200;
        break;
    }
    case (Motor_DM_Control_Method_NORMAL_EMIT):
    {
        CAN_Tx_ID = __CAN_Tx_ID + 0x300;
        break;
    }
    }
    Motor_DM_Control_Method = __Motor_DM_Control_Method;
    Angle_Max = __Angle_Max;
    Omega_Max = __Omega_Max;
    Torque_Max = __Torque_Max;
    Current_Max = __Current_Max;
}

/**
 * @brief CAN通信接收回调函数
 *
 */
void Class_Motor_DM_Normal::CAN_RxCpltCallback()
{
    // 滑动窗口, 判断电机是否在线
    Flag += 1;

    Data_Process();
}

/**
 * @brief 发送清除错误信息
 *
 */
void Class_Motor_DM_Normal::CAN_Send_Clear_Error() const
{
    CAN_Transmit_Data(CAN_Manage_Object->CAN_Handler, CAN_Tx_ID, DM_Motor_CAN_Message_Clear_Error, 8);
}

/**
 * @brief 发送使能电机
 *
 */
void Class_Motor_DM_Normal::CAN_Send_Enter() const
{
    CAN_Transmit_Data(CAN_Manage_Object->CAN_Handler, CAN_Tx_ID, DM_Motor_CAN_Message_Enter, 8);
}

/**
 * @brief 发送失能电机
 *
 */
void Class_Motor_DM_Normal::CAN_Send_Exit() const
{
    CAN_Transmit_Data(CAN_Manage_Object->CAN_Handler, CAN_Tx_ID, DM_Motor_CAN_Message_Exit, 8);
}

/**
 * @brief 发送保存当前位置为零点
 *
 */
void Class_Motor_DM_Normal::CAN_Send_Save_Zero() const
{
    CAN_Transmit_Data(CAN_Manage_Object->CAN_Handler, CAN_Tx_ID, DM_Motor_CAN_Message_Save_Zero, 8);
}

/**
 * @brief TIM定时器中断定期检测电机是否存活, 检测周期取决于电机掉线时长
 *
 */
void Class_Motor_DM_Normal::TIM_100ms_Alive_PeriodElapsedCallback()
{
    // 判断该时间段内是否接收过电机数据
    if (Flag == Pre_Flag)
    {
        // 电机断开连接
        Motor_DM_Status = Motor_DM_Status_DISABLE;
    }
    else
    {
        // 电机保持连接
        Motor_DM_Status = Motor_DM_Status_ENABLE;
    }

    Pre_Flag = Flag;

    if (Motor_DM_Status == Motor_DM_Status_DISABLE)
    {
        CAN_Send_Enter();
    }
}

/**
 * @brief TIM定时器中断发送出去的回调函数, 计算周期取决于自主设置的控制周期
 *
 */
void Class_Motor_DM_Normal::TIM_Send_PeriodElapsedCallback()
{
    if (Rx_Data.Control_Status == Motor_DM_Control_Status_ENABLE)
    {
        // 电机在线, 正常控制
        Basic_Math_Constrain(&Control_Angle, -Angle_Max, Angle_Max);
        Basic_Math_Constrain(&Control_Omega, -Omega_Max, Omega_Max);
        Basic_Math_Constrain(&Control_Torque, -Torque_Max, Torque_Max);
        Basic_Math_Constrain(&Control_Current, -Current_Max, Current_Max);
        Basic_Math_Constrain(&K_P, 0.0f, 500.0f);
        Basic_Math_Constrain(&K_D, 0.0f, 5.0f);

        Output();
    }
    else
    {
        if (Rx_Data.Control_Status == Motor_DM_Control_Status_DISABLE)
        {
            // 电机可能掉线, 使能电机
            CAN_Send_Enter();
        }
        else
        {
            // 电机错误, 发送清除错误帧, 使能电机
            CAN_Send_Clear_Error();
            CAN_Send_Enter();
        }
    }
}

/**
 * @brief 数据处理过程
 *
 */
void Class_Motor_DM_Normal::Data_Process()
{
    // 数据处理过程
    uint16_t tmp_encoder, tmp_omega, tmp_torque;
    Struct_Motor_DM_CAN_Rx_Data_Normal *tmp_buffer = (Struct_Motor_DM_CAN_Rx_Data_Normal *) CAN_Manage_Object->Rx_Buffer;

    // 电机ID不匹配, 则不进行处理
    if (tmp_buffer->CAN_ID != (CAN_Tx_ID & 0x0f))
    {
        return;
    }

    // 处理大小端
    Basic_Math_Endian_Reverse_16((void *) &tmp_buffer->Angle_Reverse, &tmp_encoder);
    tmp_omega = (tmp_buffer->Omega_11_4 << 4) | (tmp_buffer->Omega_3_0_Torque_11_8 >> 4);
    tmp_torque = ((tmp_buffer->Omega_3_0_Torque_11_8 & 0x0f) << 8) | (tmp_buffer->Torque_7_0);

    Rx_Data.Control_Status = static_cast<Enum_Motor_DM_Control_Status_Normal>(tmp_buffer->Control_Status_Enum);

    // 计算电机本身信息
    Rx_Data.Now_Angle = Basic_Math_Int_To_Float(tmp_encoder, 0x7fff, (1 << 16) - 1, 0, Angle_Max);
    Rx_Data.Now_Omega = Basic_Math_Int_To_Float(tmp_omega, 0x7ff, (1 << 12) - 1, 0, Omega_Max);
    Rx_Data.Now_Torque = Basic_Math_Int_To_Float(tmp_torque, 0x7ff, (1 << 12) - 1, 0, Torque_Max);
    // 达妙反馈温度原始单位就是摄氏度，设备层统一向上返回摄氏度。
    Rx_Data.Now_MOS_Temperature = tmp_buffer->MOS_Temperature;
    Rx_Data.Now_Rotor_Temperature = tmp_buffer->Rotor_Temperature;
}

/**
 * @brief 电机数据输出到CAN总线
 *
 */
void Class_Motor_DM_Normal::Output()
{
    // 电机控制
    switch (Motor_DM_Control_Method)
    {
    case (Motor_DM_Control_Method_NORMAL_MIT):
    {
        Struct_Motor_DM_CAN_Tx_Data_Normal_MIT *tmp_buffer = (Struct_Motor_DM_CAN_Tx_Data_Normal_MIT *) Tx_Data;

        uint16_t tmp_angle, tmp_omega, tmp_torque, tmp_k_p, tmp_k_d;

        tmp_angle = Basic_Math_Float_To_Int(Control_Angle, 0, Angle_Max, 0x7fff, (1 << 16) - 1);
        tmp_omega = Basic_Math_Float_To_Int(Control_Omega, 0, Omega_Max, 0x7ff, (1 << 12) - 1);
        tmp_torque = Basic_Math_Float_To_Int(Control_Torque, 0, Torque_Max, 0x7ff, (1 << 12) - 1);
        tmp_k_p = Basic_Math_Float_To_Int(K_P, 0, 500.0f, 0, (1 << 12) - 1);
        tmp_k_d = Basic_Math_Float_To_Int(K_D, 0, 5.0f, 0, (1 << 12) - 1);

        tmp_buffer->Control_Angle_Reverse = Basic_Math_Endian_Reverse_16(&tmp_angle, nullptr);
        tmp_buffer->Control_Omega_11_4 = tmp_omega >> 4;
        tmp_buffer->Control_Omega_3_0_K_P_11_8 = ((tmp_omega & 0x0f) << 4) | (tmp_k_p >> 8);
        tmp_buffer->K_P_7_0 = tmp_k_p & 0xff;
        tmp_buffer->K_D_11_4 = tmp_k_d >> 4;
        tmp_buffer->K_D_3_0_Control_Torque_11_8 = ((tmp_k_d & 0x0f) << 4) | (tmp_torque >> 8);
        tmp_buffer->Control_Torque_7_0 = tmp_torque & 0xff;

        CAN_Transmit_Data(CAN_Manage_Object->CAN_Handler, CAN_Tx_ID, Tx_Data, 8);

        break;
    }
    case (Motor_DM_Control_Method_NORMAL_ANGLE_OMEGA):
    {
        Struct_Motor_DM_CAN_Tx_Data_Normal_Angle_Omega *tmp_buffer = (Struct_Motor_DM_CAN_Tx_Data_Normal_Angle_Omega *) Tx_Data;

        tmp_buffer->Control_Angle = Control_Angle;
        tmp_buffer->Control_Omega = Control_Omega;

        CAN_Transmit_Data(CAN_Manage_Object->CAN_Handler, CAN_Tx_ID, Tx_Data, 8);

        break;
    }
    case (Motor_DM_Control_Method_NORMAL_OMEGA):
    {
        Struct_Motor_DM_CAN_Tx_Data_Normal_Omega *tmp_buffer = (Struct_Motor_DM_CAN_Tx_Data_Normal_Omega *) Tx_Data;

        tmp_buffer->Control_Omega = Control_Omega;

        CAN_Transmit_Data(CAN_Manage_Object->CAN_Handler, CAN_Tx_ID, Tx_Data, 4);

        break;
    }
    case (Motor_DM_Control_Method_NORMAL_EMIT):
    {
        Struct_Motor_DM_CAN_Tx_Data_Normal_EMIT *tmp_buffer = (Struct_Motor_DM_CAN_Tx_Data_Normal_EMIT *) Tx_Data;

        tmp_buffer->Control_Angle = Control_Angle;
        tmp_buffer->Control_Omega = (uint16_t)(Control_Omega * 100.0f);
        tmp_buffer->Control_Current = (uint16_t)(Control_Current / Current_Max * 10000.0f);

        CAN_Transmit_Data(CAN_Manage_Object->CAN_Handler, CAN_Tx_ID, Tx_Data, 8);

        break;
    }
    }
}

/**
 * @brief 电机初始化
 * 
 * @param hcan 绑定的CAN总线
 * @param __CAN_Rx_ID 绑定的CAN ID
 * @param __Motor_DM_Control_Method 电机控制方式, 默认角度
 * @param __Encoder_Offset 编码器偏移, 默认0
 * @param __Nearest_Angle 就近转位的单次最大旋转角度, 其数值一般为圆周的整数倍或纯小数倍, 且纯小数倍时可均分圆周, 0表示不启用就近转位
 * @param __Gearbox_Rate 减速比, 默认10.0f
 */
void Class_Motor_DM_1_To_4::Init(const FDCAN_HandleTypeDef *hcan, const Enum_Motor_DM_Motor_ID_1_To_4 &__CAN_Rx_ID, const Enum_Motor_DM_Control_Method &__Motor_DM_Control_Method, const int32_t &__Encoder_Offset, const float &__Nearest_Angle, const float &__Gearbox_Rate)
{
    if (hcan->Instance == FDCAN1)
    {
        CAN_Manage_Object = &CAN1_Manage_Object;
    }
    else if (hcan->Instance == FDCAN2)
    {
        CAN_Manage_Object = &CAN2_Manage_Object;
    }
    else if (hcan->Instance == FDCAN3)
    {
        CAN_Manage_Object = &CAN3_Manage_Object;
    }
    CAN_Rx_ID = __CAN_Rx_ID;
    Motor_DM_Control_Method = __Motor_DM_Control_Method;
    Encoder_Offset = __Encoder_Offset;
    Nearest_Angle = __Nearest_Angle;
    Gearbox_Rate = __Gearbox_Rate;
    Tx_Data = allocate_tx_data(hcan, __CAN_Rx_ID);
}

/**
 * @brief CAN通信接收回调函数
 *
 */
void Class_Motor_DM_1_To_4::CAN_RxCpltCallback()
{
    // 滑动窗口, 判断电机是否在线
    Flag += 1;

    Data_Process();
}

/**
 * @brief TIM定时器中断定期检测电机是否存活
 *
 */
void Class_Motor_DM_1_To_4::TIM_100ms_Alive_PeriodElapsedCallback()
{
    // 判断该时间段内是否接收过电机数据
    if (Flag == Pre_Flag)
    {
        // 电机断开连接
        Motor_DM_Status = Motor_DM_Status_DISABLE;
        PID_Angle.Set_Integral_Error(0.0f);
        PID_Omega.Set_Integral_Error(0.0f);
    }
    else
    {
        // 电机保持连接
        Motor_DM_Status = Motor_DM_Status_ENABLE;
    }
    Pre_Flag = Flag;
}

/**
 * @brief TIM定时器中断计算回调函数, 计算周期取决于电机反馈周期
 *
 */
void Class_Motor_DM_1_To_4::TIM_Calculate_PeriodElapsedCallback()
{
    PID_Calculate();

    Out = (Target_Torque + Feedforward_Torque) / CURRENT_TO_TORQUE / Gearbox_Rate * CURRENT_TO_OUT;
    Basic_Math_Constrain(&Out, -OUT_MAX, OUT_MAX);

    Output();

    Feedforward_Torque = 0.0f;
    Feedforward_Omega = 0.0f;
}

/**
 * @brief 数据处理过程
 *
 */
void Class_Motor_DM_1_To_4::Data_Process()
{
    // 数据处理过程
    int16_t delta_encoder;
    uint16_t tmp_encoder;
    int16_t tmp_omega, tmp_current;
    Struct_Motor_DM_CAN_Rx_Data_1_To_4 *tmp_buffer = (Struct_Motor_DM_CAN_Rx_Data_1_To_4 *) CAN_Manage_Object->Rx_Buffer;

    // 处理大小端
    Basic_Math_Endian_Reverse_16((void *) &tmp_buffer->Encoder_Reverse, (void *) &tmp_encoder);
    Basic_Math_Endian_Reverse_16((void *) &tmp_buffer->Omega_Reverse, (void *) &tmp_omega);
    Basic_Math_Endian_Reverse_16((void *) &tmp_buffer->Current_Reverse, (void *) &tmp_current);

    // 计算圈数与总编码器值
    delta_encoder = tmp_encoder - Rx_Data.Pre_Encoder;
    if (delta_encoder < -ENCODER_NUM_PER_ROUND / 2)
    {
        // 正方向转过了一圈
        Rx_Data.Total_Round++;
    }
    else if (delta_encoder > ENCODER_NUM_PER_ROUND / 2)
    {
        // 反方向转过了一圈
        Rx_Data.Total_Round--;
    }
    Rx_Data.Total_Encoder = Rx_Data.Total_Round * ENCODER_NUM_PER_ROUND + tmp_encoder + Encoder_Offset;

    // 计算电机本身信息
    Rx_Data.Now_Angle = (float) Rx_Data.Total_Encoder / (float) ENCODER_NUM_PER_ROUND * 2.0f * PI;
    Rx_Data.Now_Omega = tmp_omega / 100.0f * BASIC_MATH_RPM_TO_RADPS;
    Rx_Data.Now_Torque = tmp_current / 1000.0f * CURRENT_TO_TORQUE * Gearbox_Rate;
    Rx_Data.Now_MOS_Temperature = tmp_buffer->MOS_Temperature;
    Rx_Data.Now_Rotor_Temperature = tmp_buffer->Rotor_Temperature;

    // 存储预备信息
    Rx_Data.Pre_Encoder = tmp_encoder;
}

/**
 * @brief 计算PID
 *
 */
void Class_Motor_DM_1_To_4::PID_Calculate()
{
    switch (Motor_DM_Control_Method)
    {
    case (Motor_DM_Control_Method_1_TO_4_CURRENT):
    {
        break;
    }
    case (Motor_DM_Control_Method_1_TO_4_OMEGA):
    {
        PID_Omega.Set_Target(Target_Omega + Feedforward_Omega);
        PID_Omega.Set_Now(Rx_Data.Now_Omega);
        PID_Omega.TIM_Calculate_PeriodElapsedCallback();

        Target_Torque = PID_Omega.Get_Out();

        break;
    }
    case (Motor_DM_Control_Method_1_TO_4_ANGLE):
    {
        PID_Angle.Set_Target(Target_Angle);
        PID_Angle.Set_Now(Rx_Data.Now_Angle);
        PID_Angle.TIM_Calculate_PeriodElapsedCallback();

        Target_Omega = PID_Angle.Get_Out();

        PID_Omega.Set_Target(Target_Omega + Feedforward_Omega);
        PID_Omega.Set_Now(Rx_Data.Now_Omega);
        PID_Omega.TIM_Calculate_PeriodElapsedCallback();

        Target_Torque = PID_Omega.Get_Out();

        break;
    }
    }
}

/**
 * @brief 电机数据输出到CAN总线发送缓冲区
 *
 */
void Class_Motor_DM_1_To_4::Output()
{
    *(int16_t *) Tx_Data = (int16_t)(Out);
}

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/

/**
 * @section DM_MIT_Position_Control_Impl MIT 位置闭环实现
 * @brief 达妙电机 MIT 模式位置闭环控制封装实现
 * @version 1.0
 *
 * @copyright USTC-RoboWalker (c) 2026
 *
 */

/* Function prototypes -------------------------------------------------------*/

/**
 * @brief 初始化达妙电机 MIT 位置闭环控制器
 *
 * @param hcan 绑定的CAN总线
 * @param __CAN_Rx_ID 收数据绑定的CAN ID(主机ID, 与上位机Master_ID一致), 默认0x00
 * @param __CAN_Tx_ID 发数据绑定的CAN ID(电机ID+控制模式偏移), 默认0x01
 * @param __Angle_Max 最大位置(与上位机PMAX一致), rad, 默认12.5
 * @param __Omega_Max 最大速度(与上位机VMAX一致), rad/s, 默认30
 * @param __Torque_Max 最大扭矩(与上位机TMAX一致), N·m, 默认10
 * @param __Config 控制参数(默认与 Zephyr 样例一致)
 */
void Class_Motor_DM_MIT_Position_Control::Init(const FDCAN_HandleTypeDef *hcan, const uint8_t &__CAN_Rx_ID, const uint8_t &__CAN_Tx_ID, const float &__Angle_Max, const float &__Omega_Max, const float &__Torque_Max, const Struct_Motor_DM_MIT_Position_Control_Config &__Config)
{
    Config = __Config;

    // 电机作为传感器+执行器: MIT 模式, 仅组扭矩指令帧发送, 下发给电机内部扭矩环执行
    Motor.Init(hcan, __CAN_Rx_ID, __CAN_Tx_ID, Motor_DM_Control_Method_NORMAL_MIT, __Angle_Max, __Omega_Max, __Torque_Max);
    // 软件闭环不使用电机内部伺服, 电机内部位置/速度增益置 0
    Motor.Set_K_P(0.0f);
    Motor.Set_K_D(0.0f);
    Motor.Set_Control_Torque(0.0f);

    // 位置环 PID: 输出 = 速度参考(rad/s)
    PID_Angle.Init(Config.Position_KP, Config.Position_KI, Config.Position_KD, 0.0f,
                   Config.Position_Integral_Max,     // 积分限幅
                   Config.Position_Out_Max_RadS,     // 输出限幅(速度参考上限)
                   Config.Control_Period_S,          // D_T
                   Config.Position_Dead_Zone_Rad);   // 位置死区

    // 速度环 PID: 输出 = 扭矩(N·m)
    PID_Omega.Init(Config.Velocity_KP, Config.Velocity_KI, Config.Velocity_KD, 0.0f,
                   Config.Velocity_Integral_Max_Nm,  // 积分限幅(扭矩)
                   Config.Torque_Max_Nm,             // 输出限幅(扭矩软限)
                   Config.Control_Period_S,
                   Config.Velocity_Dead_Zone_RadS);  // 速度软死区

    // 状态清零
    Cnt_1ms = 0;
    Pre_Velocity_Reference = 0.0f;
    Filtered_Omega = 0.0f;
    Filter_Initialized = false;
    Zero_Initialized = false;
    Zero_Target_Rad = 0.0f;
    Sample_Target_Enable = true;
    Target_Angle_Manually = 0.0f;
    Protection_Tripped = false;
    Limit_Gate_Blocked = false;
    Output_Enable = true;
    Enabled_Elapsed_Ms = 0;
    Target_Angle_Rad = 0.0f;
    Now_Angle_Rad = 0.0f;
    Velocity_Reference_RadS = 0.0f;
    Now_Omega_RadS = 0.0f;
    Target_Torque_Nm = 0.0f;
    Now_Torque_Nm = 0.0f;
    Now_MOS_Temperature_C = 0.0f;
    Now_Rotor_Temperature_C = 0.0f;
}
/**
 * @brief 1ms 周期回调, 内部按 Control_Period(5ms)节拍执行位置闭环
 *
 * @param elapsed_ms 从使能起的运行时长, ms
 */
void Class_Motor_DM_MIT_Position_Control::TIM_1ms_PeriodElapsedCallback(const int64_t &elapsed_ms)
{
    Cnt_1ms++;
    if (Cnt_1ms < (uint32_t) (Config.Control_Period_S * 1000.0f))
    {
        return;
    }
    Cnt_1ms = 0;
    const float dt = Config.Control_Period_S;

    // 反馈
    const float now_angle = Motor.Get_Now_Angle();
    const float now_omega = Motor.Get_Now_Omega();
    const float now_mos_temp = Motor.Get_Now_MOS_Temperature();
    const float now_rotor_temp = Motor.Get_Now_Rotor_Temperature();

    // 输出使能与反馈在线保护: 任一不满足时下发 0 扭矩，并清空控制器历史状态。
    if (!Output_Enable || Motor.Get_Status() != Motor_DM_Status_ENABLE)
    {
        Motor.Set_Control_Torque(0.0f);
        Motor.TIM_Send_PeriodElapsedCallback();

        PID_Angle.Set_Integral_Error(0.0f);
        PID_Omega.Set_Integral_Error(0.0f);
        Pre_Velocity_Reference = 0.0f;
        Filtered_Omega = 0.0f;
        Filter_Initialized = false;
        Enabled_Elapsed_Ms = 0;
        Protection_Tripped = false;
        Limit_Gate_Blocked = false;

        Now_Angle_Rad = now_angle;
        Now_Omega_RadS = now_omega;
        Now_Torque_Nm = Motor.Get_Now_Torque();
        Now_MOS_Temperature_C = now_mos_temp;
        Now_Rotor_Temperature_C = now_rotor_temp;
        return;
    }

    Enabled_Elapsed_Ms += (int64_t) (Config.Control_Period_S * 1000.0f);

    // 0. 超速/超温保护(样例: 10 rad/s 与 60°C 保护)
    const float max_temp = (now_mos_temp > now_rotor_temp) ? now_mos_temp : now_rotor_temp;
    if (Basic_Math_Abs(now_omega) > Config.Velocity_Cutoff_RadS || max_temp > Config.Temperature_Cutoff_C)
    {
        Protection_Tripped = true;
        Motor.Set_Control_Torque(0.0f);
        Motor.TIM_Send_PeriodElapsedCallback();

        PID_Angle.Set_Integral_Error(0.0f);
        PID_Omega.Set_Integral_Error(0.0f);
        Pre_Velocity_Reference = 0.0f;
        Filtered_Omega = 0.0f;
        Filter_Initialized = false;

        Now_Angle_Rad = now_angle;
        Now_Omega_RadS = now_omega;
        Now_Torque_Nm = Motor.Get_Now_Torque();
        Now_MOS_Temperature_C = now_mos_temp;
        Now_Rotor_Temperature_C = now_rotor_temp;
        return;
    }
    Protection_Tripped = false;

    // 当前角度处于机械范围外时完全禁止扭矩输出；人工送回范围内后自动恢复闭环。
    const bool limit_mode = (Config.Position_Max_Rad > Config.Position_Min_Rad);
    const bool limit_gate_blocked = Config.Limit_Gate_Enable && limit_mode &&
                                    (now_angle < Config.Position_Min_Rad - Config.Limit_Gate_Tolerance_Rad ||
                                     now_angle > Config.Position_Max_Rad + Config.Limit_Gate_Tolerance_Rad);
    if (limit_gate_blocked)
    {
        Limit_Gate_Blocked = true;
        Motor.Set_Control_Torque(0.0f);
        Motor.TIM_Send_PeriodElapsedCallback();

        PID_Angle.Set_Integral_Error(0.0f);
        PID_Omega.Set_Integral_Error(0.0f);
        Pre_Velocity_Reference = 0.0f;
        Filtered_Omega = 0.0f;
        Filter_Initialized = false;

        Target_Angle_Rad = Basic_Math_Constrain(Target_Angle_Manually, Config.Position_Min_Rad, Config.Position_Max_Rad);
        Now_Angle_Rad = now_angle;
        Velocity_Reference_RadS = 0.0f;
        Now_Omega_RadS = now_omega;
        Target_Torque_Nm = 0.0f;
        Now_Torque_Nm = Motor.Get_Now_Torque();
        Now_MOS_Temperature_C = now_mos_temp;
        Now_Rotor_Temperature_C = now_rotor_temp;
        return;
    }
    Limit_Gate_Blocked = false;

    // 1. 初始零位对齐(样例: 取正方向的下一个编码器零位, 零位附近允许 Zero_Tolerance)
    if (!Zero_Initialized)
    {
        const float initial = now_angle;
        float phase = fmodf(initial, MOTOR_DM_CTRL_TWO_PI);
        if (phase < 0.0f)
        {
            phase += MOTOR_DM_CTRL_TWO_PI;
        }
        Zero_Target_Rad = initial - phase + ((phase <= Config.Zero_Tolerance_Rad) ? 0.0f : MOTOR_DM_CTRL_TWO_PI);
        Zero_Initialized = true;
    }

    // 2. 目标角度(内置序列 或 手动目标)
    float target = Sample_Target_Enable ? Target_Angle_From_Elapsed_Time(Enabled_Elapsed_Ms) : Target_Angle_Manually;
    if (limit_mode)
    {
        target = Basic_Math_Constrain(target, Config.Position_Min_Rad, Config.Position_Max_Rad);
    }

    // 3. 位置环 PID -> 速度参考(rad/s)
    PID_Angle.Set_Target(target);
    PID_Angle.Set_Now(now_angle);
    PID_Angle.TIM_Calculate_PeriodElapsedCallback();
    float velocity_ref = PID_Angle.Get_Out();

    // 4. 速度参考斜坡限制(样例 2 rad/s^2)
    if (Config.Velocity_Ramp_RadS2 > 0.0f)
    {
        const float max_delta = Config.Velocity_Ramp_RadS2 * dt;
        float delta = velocity_ref - Pre_Velocity_Reference;
        if (delta > max_delta)
        {
            delta = max_delta;
        }
        else if (delta < -max_delta)
        {
            delta = -max_delta;
        }
        velocity_ref = Pre_Velocity_Reference + delta;
    }

    // 5. 速度测量低通滤波(样例 tau = 0.02s), 首拍用测量值初始化
    if (!Filter_Initialized)
    {
        Filtered_Omega = now_omega;
        Filter_Initialized = true;
    }
    else if (Config.Velocity_Filter_Tau_S > 0.0f)
    {
        const float alpha = dt / (Config.Velocity_Filter_Tau_S + dt);
        Filtered_Omega += alpha * (now_omega - Filtered_Omega);
    }
    else
    {
        Filtered_Omega = now_omega;
    }

    // 6. 速度环 PID -> 扭矩(N·m)
    PID_Omega.Set_Target(velocity_ref);
    PID_Omega.Set_Now(Filtered_Omega);
    PID_Omega.TIM_Calculate_PeriodElapsedCallback();
    float torque = PID_Omega.Get_Out();
    // 扭矩软限(样例 kSoftwareTorqueAbsMaxNm)
    if (torque > Config.Torque_Max_Nm)
    {
        torque = Config.Torque_Max_Nm;
    }
    else if (torque < -Config.Torque_Max_Nm)
    {
        torque = -Config.Torque_Max_Nm;
    }

    // 7. 下发 MIT 扭矩指令并发送
    Motor.Set_Control_Torque(torque);
    Motor.TIM_Send_PeriodElapsedCallback();

    // 8. 遥测
    Target_Angle_Rad = target;
    Now_Angle_Rad = now_angle;
    Velocity_Reference_RadS = velocity_ref;
    Now_Omega_RadS = now_omega;
    Target_Torque_Nm = torque;
    Now_Torque_Nm = Motor.Get_Now_Torque();
    Now_MOS_Temperature_C = now_mos_temp;
    Now_Rotor_Temperature_C = now_rotor_temp;
    Pre_Velocity_Reference = velocity_ref;
}

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
