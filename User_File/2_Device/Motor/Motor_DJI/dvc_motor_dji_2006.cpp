/** @file dvc_motor_dji_2006.cpp
 * @brief M2006/C610 驱动与速度闭环实现
 * @copyright USTC-RoboWalker (c) 2023-2026
 */
#include "dvc_motor_dji_2006.h"

/**
 * @brief 电机初始化
 *
 * @param hcan CAN编号
 * @param __CAN_Rx_ID CAN ID
 * @param __Motor_DJI_Control_Method 电机控制方式, 默认角度
 * @param __Gearbox_Rate 减速箱减速比, 默认为原装减速箱, 如拆去减速箱则该值设为1
 * @param __Current_Max 最大电流
 */
void Class_Motor_DJI_C610::Init(const FDCAN_HandleTypeDef *hcan, const Enum_Motor_DJI_ID &__CAN_Rx_ID, const Enum_Motor_DJI_Control_Method &__Motor_DJI_Control_Method, const float &__Gearbox_Rate)
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
    Motor_DJI_Control_Method = __Motor_DJI_Control_Method;
    Gearbox_Rate = __Gearbox_Rate;
    Tx_Data = allocate_tx_data(hcan, __CAN_Rx_ID);
}

/**
 * @brief CAN通信接收回调函数
 *
 */
void Class_Motor_DJI_C610::CAN_RxCpltCallback()
{
    // 滑动窗口, 判断电机是否在线
    Flag += 1;

    Data_Process();
}

/**
 * @brief TIM定时器中断定期检测电机是否存活
 *
 */
void Class_Motor_DJI_C610::TIM_100ms_Alive_PeriodElapsedCallback()
{
    // 判断该时间段内是否接收过电机数据
    if (Flag == Pre_Flag)
    {
        // 电机断开连接
        Motor_DJI_Status = Motor_DJI_Status_DISABLE;
        PID_Angle.Set_Integral_Error(0.0f);
        PID_Omega.Set_Integral_Error(0.0f);
    }
    else
    {
        // 电机保持连接
        Motor_DJI_Status = Motor_DJI_Status_ENABLE;
    }
    Pre_Flag = Flag;
}

/**
 * @brief TIM定时器中断计算回调函数, 计算周期取决于电机反馈周期
 *
 */
void Class_Motor_DJI_C610::TIM_Calculate_PeriodElapsedCallback()
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
void Class_Motor_DJI_C610::Data_Process()
{
    // 数据处理过程
    int16_t delta_encoder;
    uint16_t tmp_encoder;
    int16_t tmp_omega, tmp_current;
    Struct_Motor_DJI_CAN_Rx_Data *tmp_buffer = (Struct_Motor_DJI_CAN_Rx_Data *) CAN_Manage_Object->Rx_Buffer;

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
    Rx_Data.Total_Encoder = Rx_Data.Total_Round * ENCODER_NUM_PER_ROUND + tmp_encoder;

    // 计算电机本身信息
    Rx_Data.Now_Angle = (float) Rx_Data.Total_Encoder / (float) ENCODER_NUM_PER_ROUND * 2.0f * PI / Gearbox_Rate;
    Rx_Data.Now_Omega = (float) tmp_omega * BASIC_MATH_RPM_TO_RADPS / Gearbox_Rate;
    Rx_Data.Now_Torque = tmp_current / CURRENT_TO_OUT * CURRENT_TO_TORQUE * Gearbox_Rate;
    Rx_Data.Now_Temperature = tmp_buffer->Temperature + BASIC_MATH_CELSIUS_TO_KELVIN;

    // 存储预备信息
    Rx_Data.Pre_Encoder = tmp_encoder;
}

/**
 * @brief 计算PID
 *
 */
void Class_Motor_DJI_C610::PID_Calculate()
{
    switch (Motor_DJI_Control_Method)
    {
    case (Motor_DJI_Control_Method_TORQUE):
    {
        break;
    }
    case (Motor_DJI_Control_Method_OMEGA):
    {
        PID_Omega.Set_Target(Target_Omega + Feedforward_Omega);
        PID_Omega.Set_Now(Rx_Data.Now_Omega);
        PID_Omega.TIM_Calculate_PeriodElapsedCallback();

        Target_Torque = PID_Omega.Get_Out();

        break;
    }
    case (Motor_DJI_Control_Method_ANGLE):
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
void Class_Motor_DJI_C610::Output()
{
    Tx_Data[0] = (int16_t) Out >> 8;
    Tx_Data[1] = (int16_t) Out;
}

/**
 * @copyright USTC-RoboWalker (c) 2026
 */
/* ===========================================================================
 * 3. M2006 速度闭环控制
 * =========================================================================== */

void Class_Motor_DJI_M2006_Speed_Control::Init(const FDCAN_HandleTypeDef *hcan,
                                                const Enum_Motor_DJI_ID &__CAN_Rx_ID,
                                                const float &__Gearbox_Rate,
                                                const Struct_Motor_DJI_M2006_Speed_Control_Config &__Config)
{
    Config = __Config;

    // C610 按扭矩方式工作；闭环在本封装中计算电流后换算为输出轴扭矩
    Motor.Init(hcan, __CAN_Rx_ID, Motor_DJI_Control_Method_TORQUE, __Gearbox_Rate);

    PID_Omega.Init(Config.Velocity_KP, Config.Velocity_KI, Config.Velocity_KD, 0.0f,
                   Config.Velocity_Integral_Max_A,
                   Config.Current_Max_A,
                   Config.Control_Period_S,
                   Config.Velocity_Dead_Zone_RadS);

    Cnt_1ms = 0;
    Pre_Velocity_Reference = 0.0f;
    Filtered_Omega = 0.0f;
    Filter_Initialized = false;
    Sample_Target_Enable = true;
    Target_Omega_Manually = 0.0f;
    Output_Enable = true;
    Velocity_Reference_RadS = 0.0f;
    Now_Omega_RadS = 0.0f;
    Filtered_Omega_RadS = 0.0f;
    Target_Current_A = 0.0f;
    Now_Torque_Nm = 0.0f;
}

void Class_Motor_DJI_M2006_Speed_Control::TIM_1ms_PeriodElapsedCallback(const int64_t &elapsed_ms)
{
    Cnt_1ms++;
    if (Cnt_1ms < (uint32_t) (Config.Control_Period_S * 1000.0f))
    {
        return;
    }
    Cnt_1ms = 0;

    const float dt = Config.Control_Period_S;
    const float now_omega = Motor.Get_Now_Omega();

    // 无反馈或任务层未使能时均禁止出力，避免使用过期速度闭环
    if (!Output_Enable || Motor.Get_Status() != Motor_DJI_Status_ENABLE)
    {
        Motor.Set_Target_Torque(0.0f);
        Motor.TIM_Calculate_PeriodElapsedCallback();

        PID_Omega.Set_Integral_Error(0.0f);
        Pre_Velocity_Reference = 0.0f;
        Filter_Initialized = false;

        Velocity_Reference_RadS = 0.0f;
        Now_Omega_RadS = now_omega;
        Filtered_Omega_RadS = 0.0f;
        Target_Current_A = 0.0f;
        Now_Torque_Nm = Motor.Get_Now_Torque();
        return;
    }

    const float target = Sample_Target_Enable ? Target_Omega_From_Elapsed_Time(elapsed_ms) : Target_Omega_Manually;

    float velocity_ref = target;
    if (Config.Velocity_Ramp_RadS2 > 0.0f)
    {
        const float max_delta = Config.Velocity_Ramp_RadS2 * dt;
        float delta = target - Pre_Velocity_Reference;
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

    PID_Omega.Set_Target(velocity_ref);
    PID_Omega.Set_Now(Filtered_Omega);
    PID_Omega.TIM_Calculate_PeriodElapsedCallback();
    float current = PID_Omega.Get_Out() + Config.Velocity_Feedforward_K_Static * velocity_ref;

    // 同时遵守用户软限幅和 C610 的 10 A 物理/协议上限
    float current_max = Config.Current_Max_A;
    if (current_max < 0.0f)
    {
        current_max = -current_max;
    }
    if (current_max > 10.0f)
    {
        current_max = 10.0f;
    }
    Basic_Math_Constrain(&current, -current_max, current_max);

    Motor.Set_Target_Torque(current * Config.Current_To_Torque_Nm_A);
    Motor.TIM_Calculate_PeriodElapsedCallback();

    Velocity_Reference_RadS = velocity_ref;
    Now_Omega_RadS = now_omega;
    Filtered_Omega_RadS = Filtered_Omega;
    Target_Current_A = current;
    Now_Torque_Nm = Motor.Get_Now_Torque();
    Pre_Velocity_Reference = velocity_ref;
}

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
