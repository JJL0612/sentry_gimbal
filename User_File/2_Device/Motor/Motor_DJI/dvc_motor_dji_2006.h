/**
 * @file dvc_motor_dji_2006.h
 * @brief RoboMaster M2006 P36 + C610 电调驱动
 *
 * 本文件集中提供 M2006/C610 底层驱动与 M2006 速度闭环控制接口。
 *
 * M2006/C610 关键参数由 Class_Motor_DJI_C610 实现：
 * - 原装减速比 36:1
 * - 电流指令 -10000~10000 对应 -10~10 A
 * - 输出轴扭矩常数 0.18 N.m/A
 */

#ifndef DVC_MOTOR_DJI_2006_H
#define DVC_MOTOR_DJI_2006_H

#include "dvc_motor_dji.h"

/**
 * @brief Reusable, C610无刷电调, 自带电流环, 单片机控制输出电流
 *
 */
class Class_Motor_DJI_C610
{
public:
    // PID角度环控制
    Class_PID PID_Angle;
    // PID角速度环控制
    Class_PID PID_Omega;

    void Init(const FDCAN_HandleTypeDef *hcan, const Enum_Motor_DJI_ID &__CAN_Rx_ID, const Enum_Motor_DJI_Control_Method &__Motor_DJI_Control_Method = Motor_DJI_Control_Method_OMEGA, const float &__Gearbox_Rate = 36.0f);

    inline Enum_Motor_DJI_Status Get_Status() const;

    inline float Get_Now_Angle() const;

    inline float Get_Now_Omega() const;

    inline float Get_Now_Torque() const;

    inline uint8_t Get_Now_Temperature() const;

    inline Enum_Motor_DJI_Control_Method Get_Control_Method() const;

    inline float Get_Target_Angle() const;

    inline float Get_Target_Omega() const;

    inline float Get_Target_Torque() const;

    inline float Get_Feedforward_Omega() const;

    inline float Get_Feedforward_Torque() const;

    inline void Set_Control_Method(const Enum_Motor_DJI_Control_Method &__Motor_DJI_Control_Method);

    inline void Set_Target_Angle(const float &__Target_Angle);

    inline void Set_Target_Omega(const float &__Target_Omega);

    inline void Set_Target_Torque(const float &__Target_Torque);

    inline void Set_Feedforward_Omega(const float &__Feedforward_Omega);

    inline void Set_Feedforward_Torque(const float &__Feedforward_Torque);

    void CAN_RxCpltCallback();

    void TIM_100ms_Alive_PeriodElapsedCallback();

    void TIM_Calculate_PeriodElapsedCallback();

protected:
    // 初始化相关常量

    // 绑定的CAN
    Struct_CAN_Manage_Object *CAN_Manage_Object;
    // 收数据绑定的CAN ID, C6系列0x201~0x208, GM系列0x205~0x20b
    Enum_Motor_DJI_ID CAN_Rx_ID;
    // 发送缓存区
    uint8_t *Tx_Data;
    // 减速比, 默认带减速箱
    float Gearbox_Rate;

    // 常量

    // 一圈编码器刻度
    const uint16_t ENCODER_NUM_PER_ROUND = 8192;

    // 扭矩电流常数, 减速前
    const float CURRENT_TO_TORQUE = 0.18f / 36.0f;
    // 扭矩电流到输出的转化系数
    const float CURRENT_TO_OUT = 10000.0f / 10.0f;
    // 最大输出刻度
    const float OUT_MAX = 10000.0f;

    // 内部变量

    // 当前时刻的电机接收flag
    uint32_t Flag = 0;
    // 前一时刻的电机接收flag
    uint32_t Pre_Flag = 0;
    // 输出量
    float Out = 0.0f;

    // 读变量

    // 电机状态
    Enum_Motor_DJI_Status Motor_DJI_Status = Motor_DJI_Status_DISABLE;
    // 电机对外接口信息
    Struct_Motor_DJI_Rx_Data Rx_Data;

    // 写变量

    // 读写变量

    // 电机控制方式
    Enum_Motor_DJI_Control_Method Motor_DJI_Control_Method = Motor_DJI_Control_Method_OMEGA;
    // 目标的角度, rad
    float Target_Angle = 0.0f;
    // 目标的速度, rad/s
    float Target_Omega = 0.0f;
    // 目标的扭矩, Nm
    float Target_Torque = 0.0f;
    // 前馈的速度, rad/s
    float Feedforward_Omega = 0.0f;
    // 前馈的扭矩, Nm
    float Feedforward_Torque = 0.0f;

    // 内部函数

    void Data_Process();

    void PID_Calculate();

    void Output();
};
/**
 * @brief 获取电机状态
 *
 * @return Enum_Motor_DJI_Status 电机状态
 */
inline Enum_Motor_DJI_Status Class_Motor_DJI_C610::Get_Status() const
{
    return (Motor_DJI_Status);
}

/**
 * @brief 获取当前的角度, rad
 *
 * @return float 当前的角度, rad
 */
inline float Class_Motor_DJI_C610::Get_Now_Angle() const
{
    return (Rx_Data.Now_Angle);
}

/**
 * @brief 获取当前的速度, rad/s
 *
 * @return float 当前的速度, rad/s
 */
inline float Class_Motor_DJI_C610::Get_Now_Omega() const
{
    return (Rx_Data.Now_Omega);
}

/**
 * @brief 获取当前的扭矩, Nm
 *
 * @return float 当前的扭矩, Nm
 */
inline float Class_Motor_DJI_C610::Get_Now_Torque() const
{
    return (Rx_Data.Now_Torque);
}

/**
 * @brief 获取当前的温度, K
 *
 * @return uint8_t 当前的温度, K
 */
inline uint8_t Class_Motor_DJI_C610::Get_Now_Temperature() const
{
    return (Rx_Data.Now_Temperature);
}

/**
 * @brief 获取电机控制方式
 *
 * @return Enum_Motor_DJI_Control_Method 电机控制方式
 */
inline Enum_Motor_DJI_Control_Method Class_Motor_DJI_C610::Get_Control_Method() const
{
    return (Motor_DJI_Control_Method);
}

/**
 * @brief 获取目标的角度, rad
 *
 * @return float 目标的角度, rad
 */
inline float Class_Motor_DJI_C610::Get_Target_Angle() const
{
    return (Target_Angle);
}

/**
 * @brief 获取目标的速度, rad/s
 *
 * @return float 目标的速度, rad/s
 */
inline float Class_Motor_DJI_C610::Get_Target_Omega() const
{
    return (Target_Omega);
}

/**
 * @brief 获取目标的扭矩, Nm
 *
 * @return float 目标的扭矩, Nm
 */
inline float Class_Motor_DJI_C610::Get_Target_Torque() const
{
    return (Target_Torque);
}

/**
 * @brief 获取前馈的速度, rad/s
 *
 * @return float 前馈的速度, rad/s
 */
inline float Class_Motor_DJI_C610::Get_Feedforward_Omega() const
{
    return (Feedforward_Omega);
}

/**
 * @brief 获取前馈的扭矩, Nm
 *
 * @return float 前馈的扭矩, Nm
 */
inline float Class_Motor_DJI_C610::Get_Feedforward_Torque() const
{
    return (Feedforward_Torque);
}

/**
 * @brief 设定电机控制方式
 *
 * @param __Motor_DJI_Control_Method 电机控制方式
 */
inline void Class_Motor_DJI_C610::Set_Control_Method(const Enum_Motor_DJI_Control_Method &__Motor_DJI_Control_Method)
{
    Motor_DJI_Control_Method = __Motor_DJI_Control_Method;
}

/**
 * @brief 设定目标的角度, rad
 *
 * @param __Target_Angle 目标的角度, rad
 */
inline void Class_Motor_DJI_C610::Set_Target_Angle(const float &__Target_Angle)
{
    Target_Angle = __Target_Angle;
}

/**
 * @brief 设定目标的速度, rad/s
 *
 * @param __Target_Omega 目标的速度, rad/s
 */
inline void Class_Motor_DJI_C610::Set_Target_Omega(const float &__Target_Omega)
{
    Target_Omega = __Target_Omega;
}

/**
 * @brief 设定目标的扭矩, Nm
 *
 * @param __Target_Torque 目标的扭矩, Nm
 */
inline void Class_Motor_DJI_C610::Set_Target_Torque(const float &__Target_Torque)
{
    Target_Torque = __Target_Torque;
}

/**
 * @brief 设定前馈的速度, rad/s
 *
 * @param __Feedforward_Omega 前馈的速度, rad/s
 */
inline void Class_Motor_DJI_C610::Set_Feedforward_Omega(const float &__Feedforward_Omega)
{
    Feedforward_Omega = __Feedforward_Omega;
}

/**
 * @brief 设定前馈的扭矩, Nm
 *
 * @param __Feedforward_Torque 前馈的扭矩, Nm
 */
inline void Class_Motor_DJI_C610::Set_Feedforward_Torque(const float &__Feedforward_Torque)
{
    Feedforward_Torque = __Feedforward_Torque;
}

/**
 * @brief M2006 电机语义类型，复用已经过协议参数化的 C610 底层驱动
 */
class Class_Motor_DJI_M2006 : public Class_Motor_DJI_C610
{
};

/**
 * @copyright USTC-RoboWalker (c) 2026
 */
/* ---------------------------------------------------------------------------
 * 3. M2006 速度闭环控制
 *    结构与 M3508 相同: 斜坡限制 -> 测量低通 -> 速度环 PID + 前馈 -> 电流
 *    差异由 M2006/C610 参数体现: 36:1、10 A 满量程、0.18 N.m/A
 * ------------------------------------------------------------------------- */

/**
 * @brief M2006 速度闭环控制参数
 */
struct Struct_Motor_DJI_M2006_Speed_Control_Config
{
    // 速度环 PID，输出 = 电流 A
    float Velocity_KP = 0.02f;
    float Velocity_KI = 0.05f;
    float Velocity_KD = 0.0f;
    // 速度环积分限幅，A
    float Velocity_Integral_Max_A = 0.1f;
    // 静态速度前馈，A/(rad/s)
    float Velocity_Feedforward_K_Static = 0.005f;
    // 速度参考斜坡，rad/s^2
    float Velocity_Ramp_RadS2 = 100.0f;
    // 速度测量低通时间常数，s
    float Velocity_Filter_Tau_S = 0.025f;
    // 速度软死区，rad/s
    float Velocity_Dead_Zone_RadS = 0.20f;
    // 电流软限幅，A；不得大于 C610 协议/硬件上限 10 A
    float Current_Max_A = 0.8f;
    // M2006 原装减速箱输出轴扭矩常数，N.m/A
    float Current_To_Torque_Nm_A = 0.18f;
    // 控制周期，s
    float Control_Period_S = 0.005f;
};

inline Struct_Motor_DJI_M2006_Speed_Control_Config Motor_DJI_M2006_Speed_Control_Config_Default()
{
    Struct_Motor_DJI_M2006_Speed_Control_Config tmp_config;
    return (tmp_config);
}

/**
 * @brief Reusable, M2006 + C610 速度闭环控制器
 */
class Class_Motor_DJI_M2006_Speed_Control
{
public:
    Class_Motor_DJI_M2006 Motor;
    Class_PID PID_Omega;

    // 遥测读变量
    float Velocity_Reference_RadS = 0.0f;
    float Filtered_Omega_RadS = 0.0f;
    float Now_Omega_RadS = 0.0f;
    float Target_Current_A = 0.0f;
    float Now_Torque_Nm = 0.0f;

    void Init(const FDCAN_HandleTypeDef *hcan,
              const Enum_Motor_DJI_ID &__CAN_Rx_ID,
              const float &__Gearbox_Rate = 36.0f,
              const Struct_Motor_DJI_M2006_Speed_Control_Config &__Config = Motor_DJI_M2006_Speed_Control_Config_Default());

    // 在 1ms 周期任务中调用，内部按 Control_Period 节拍计算
    void TIM_1ms_PeriodElapsedCallback(const int64_t &elapsed_ms);

    inline void Set_Sample_Target_Enable(const bool &__Sample_Target_Enable);
    inline void Set_Target_Omega_Manually(const float &__Target_Omega_RadS);

    // false 时输出零电流，并清除 PID、斜坡及滤波状态
    bool Output_Enable = true;
    inline void Set_Output_Enable(const bool &__Output_Enable);
    inline bool Get_Output_Enable() const;

protected:
    Struct_Motor_DJI_M2006_Speed_Control_Config Config;
    bool Sample_Target_Enable = true;
    float Target_Omega_Manually = 0.0f;
    float Pre_Velocity_Reference = 0.0f;
    float Filtered_Omega = 0.0f;
    bool Filter_Initialized = false;
    uint32_t Cnt_1ms = 0;

    float Target_Omega_From_Elapsed_Time(const int64_t &elapsed_ms);
};

inline float Class_Motor_DJI_M2006_Speed_Control::Target_Omega_From_Elapsed_Time(const int64_t &elapsed_ms)
{
    if (elapsed_ms < 500)
    {
        return (0.0f);
    }
    if (elapsed_ms < 300000)
    {
        return (20.0f * sinf((float) elapsed_ms / 1000.0f));
    }
    return (0.0f);
}

inline void Class_Motor_DJI_M2006_Speed_Control::Set_Sample_Target_Enable(const bool &__Sample_Target_Enable)
{
    Sample_Target_Enable = __Sample_Target_Enable;
}

inline void Class_Motor_DJI_M2006_Speed_Control::Set_Target_Omega_Manually(const float &__Target_Omega_RadS)
{
    Target_Omega_Manually = __Target_Omega_RadS;
}

inline void Class_Motor_DJI_M2006_Speed_Control::Set_Output_Enable(const bool &__Output_Enable)
{
    Output_Enable = __Output_Enable;
}

inline bool Class_Motor_DJI_M2006_Speed_Control::Get_Output_Enable() const
{
    return (Output_Enable);
}

#endif

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
