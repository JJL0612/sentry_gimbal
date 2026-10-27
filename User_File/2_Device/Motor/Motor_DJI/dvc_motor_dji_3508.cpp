/**
 * @file dvc_motor_dji_3508.cpp
 * @author yssickjgd (1345578933@qq.com)
 * @brief 大疆M3508减速电机 + C620电调实现(速度环为主)
 * @version 2.1
 * @date 2024-01-01 1.1 官方6020驱动更新, 适配电压控制与电流控制
 * @date 2025-12-03 2.1 彻底抛弃6020电压控制版本, 仅保留电流控制版本; 从 dvc_motor_dji 中独立拆分
 *
 * @copyright USTC-RoboWalker (c) 2023-2025
 *
 */

/* Includes ------------------------------------------------------------------*/

#include "dvc_motor_dji_3508.h"

/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function declarations ---------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/

/**
 * @brief 电机初始化
 *
 * @param hcan CAN编号
 * @param __CAN_Rx_ID CAN ID
 * @param __Motor_DJI_Control_Method 电机控制方式, 默认速度
 * @param __Gearbox_Rate 减速箱减速比, 默认为原装减速箱, 如拆去减速箱则该值设为1
 */
void Class_Motor_DJI_C620::Init(const FDCAN_HandleTypeDef *hcan, const Enum_Motor_DJI_ID &__CAN_Rx_ID, const Enum_Motor_DJI_Control_Method &__Motor_DJI_Control_Method, const float &__Gearbox_Rate)
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
void Class_Motor_DJI_C620::CAN_RxCpltCallback()
{
    // 滑动窗口, 判断电机是否在线
    Flag += 1;

    Data_Process();
}

/**
 * @brief TIM定时器中断定期检测电机是否存活
 *
 */
void Class_Motor_DJI_C620::TIM_100ms_Alive_PeriodElapsedCallback()
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
void Class_Motor_DJI_C620::TIM_Calculate_PeriodElapsedCallback()
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
void Class_Motor_DJI_C620::Data_Process()
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
    Rx_Data.Now_Power = power_calculate(POWER_K_0, POWER_K_1, POWER_K_2, POWER_A, Rx_Data.Now_Torque / Gearbox_Rate, Rx_Data.Now_Omega * Gearbox_Rate);

    // 存储预备信息
    Rx_Data.Pre_Encoder = tmp_encoder;
}

/**
 * @brief 计算PID
 *
 */
void Class_Motor_DJI_C620::PID_Calculate()
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
    default:
    {
        Target_Torque = 0.0f;

        break;
    }
    }
}

/**
 * @brief 电机数据输出到CAN总线发送缓冲区
 *
 */
void Class_Motor_DJI_C620::Output()
{
    Tx_Data[0] = (int16_t) Out >> 8;
    Tx_Data[1] = (int16_t) Out;
}
/**
 * @brief 电机初始化
 *
 * @param hcan CAN编号
 * @param __CAN_Rx_ID CAN ID
 * @param __Motor_DJI_Control_Method 电机控制方式, 默认角速度
 * @param __Gearbox_Rate 减速箱减速比, 默认为M3508原装减速箱3591:187, 如拆去减速箱则该值设为1
 */
void Class_Motor_DJI_M3508::Init(const FDCAN_HandleTypeDef *hcan, const Enum_Motor_DJI_ID &__CAN_Rx_ID, const Enum_Motor_DJI_Control_Method &__Motor_DJI_Control_Method, const float &__Gearbox_Rate)
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
void Class_Motor_DJI_M3508::CAN_RxCpltCallback()
{
    // 滑动窗口, 判断电机是否在线
    Flag += 1;

    Data_Process();
}

/**
 * @brief TIM定时器中断定期检测电机是否存活
 *
 */
void Class_Motor_DJI_M3508::TIM_100ms_Alive_PeriodElapsedCallback()
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
void Class_Motor_DJI_M3508::TIM_Calculate_PeriodElapsedCallback()
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
void Class_Motor_DJI_M3508::Data_Process()
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
    Rx_Data.Now_Power = power_calculate(POWER_K_0, POWER_K_1, POWER_K_2, POWER_A, Rx_Data.Now_Torque / Gearbox_Rate, Rx_Data.Now_Omega * Gearbox_Rate);

    // 存储预备信息
    Rx_Data.Pre_Encoder = tmp_encoder;
}

/**
 * @brief 计算PID
 *
 */
void Class_Motor_DJI_M3508::PID_Calculate()
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
    default:
    {
        Target_Torque = 0.0f;

        break;
    }
    }
}

/**
 * @brief 电机数据输出到CAN总线发送缓冲区
 *
 */
void Class_Motor_DJI_M3508::Output()
{
    Tx_Data[0] = (int16_t) Out >> 8;
    Tx_Data[1] = (int16_t) Out;
}

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/

/**
 * @copyright USTC-RoboWalker (c) 2026
 */
/* ===========================================================================
 * 2. M3508 速度闭环控制
 * =========================================================================== */

/**
 * @brief 初始化 M3508 速度闭环控制器
 *
 * @param hcan 绑定的CAN总线
 * @param __CAN_Rx_ID 绑定的CAN ID
 * @param __Gearbox_Rate 减速箱减速比(默认 M3508 原装减速箱 3591:187, 拆减速箱设为1并同步修改 Config.Current_To_Torque_Nm_A)
 * @param __Config 控制参数(默认与 Zephyr 样例一致)
 */
void Class_Motor_DJI_M3508_Speed_Control::Init(const FDCAN_HandleTypeDef *hcan, const Enum_Motor_DJI_ID &__CAN_Rx_ID, const float &__Gearbox_Rate, const Struct_Motor_DJI_M3508_Speed_Control_Config &__Config)
{
    Config = __Config;

    // 电机作为传感器+执行器: 控制方式 TORQUE(电机类只做电流输出与 CAN 收发)
    Motor.Init(hcan, __CAN_Rx_ID, Motor_DJI_Control_Method_TORQUE, __Gearbox_Rate);

    // 速度环 PID: 输出 = 电流(A), 积分/输出限幅按样例
    PID_Omega.Init(Config.Velocity_KP, Config.Velocity_KI, Config.Velocity_KD, 0.0f,
                   Config.Velocity_Integral_Max_A,   // 积分限幅(电流)
                   Config.Current_Max_A,             // 输出限幅(电流软限)
                   Config.Control_Period_S,
                   Config.Velocity_Dead_Zone_RadS);  // 速度软死区

    // 状态清零
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

/**
 * @brief 1ms 周期回调, 内部按 Control_Period(5ms)节拍执行速度闭环
 *
 * @param elapsed_ms 从启动起的运行时长, ms
 */
void Class_Motor_DJI_M3508_Speed_Control::TIM_1ms_PeriodElapsedCallback(const int64_t &elapsed_ms)
{
    Cnt_1ms++;
    if (Cnt_1ms < (uint32_t) (Config.Control_Period_S * 1000.0f))
    {
        return;
    }
    Cnt_1ms = 0;
    const float dt = Config.Control_Period_S;
    const float now_omega = Motor.Get_Now_Omega();

    // 0. 输出使能(任务层安全链路): 未使能时不下发任何扭矩, 并清掉速度环积分/斜坡/滤波状态
    if (!Output_Enable)
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

    // 1. 目标速度(内置序列 或 手动目标)
    const float target = Sample_Target_Enable ? Target_Omega_From_Elapsed_Time(elapsed_ms) : Target_Omega_Manually;

    // 2. 速度参考斜坡限制(样例 100 rad/s^2)
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

    // 3. 速度测量低通滤波(样例 tau = 0.025s), 首拍用测量值初始化
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

    // 4. 速度环 PID -> 电流(A)
    PID_Omega.Set_Target(velocity_ref);
    PID_Omega.Set_Now(Filtered_Omega);
    PID_Omega.TIM_Calculate_PeriodElapsedCallback();
    float current = PID_Omega.Get_Out();
    // 静态前馈(样例 feedforward.k_static), 电流 = k_static * 速度参考
    current += Config.Velocity_Feedforward_K_Static * velocity_ref;
    // 电流软限幅(样例 effort_abs_max)
    if (current > Config.Current_Max_A)
    {
        current = Config.Current_Max_A;
    }
    else if (current < -Config.Current_Max_A)
    {
        current = -Config.Current_Max_A;
    }

    // 5. 电流 -> 扭矩 -> 电机输出电流刻度
    Motor.Set_Target_Torque(current * Config.Current_To_Torque_Nm_A);
    Motor.TIM_Calculate_PeriodElapsedCallback();

    // 6. 遥测
    Velocity_Reference_RadS = velocity_ref;
    Now_Omega_RadS = now_omega;
    Filtered_Omega_RadS = Filtered_Omega;
    Target_Current_A = current;
    Now_Torque_Nm = Motor.Get_Now_Torque();
    Pre_Velocity_Reference = velocity_ref;
}
