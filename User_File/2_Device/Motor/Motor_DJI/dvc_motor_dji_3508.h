/**
 * @file dvc_motor_dji_3508.h
 * @author yssickjgd (1345578933@qq.com)
 * @brief 大疆M3508减速电机 + C620电调配置与操作, 速度环为主(预留位置环)
 * @version 2.1
 * @date 2024-01-01 1.1 官方6020驱动更新, 适配电压控制与电流控制
 * @date 2025-12-03 2.1 彻底抛弃6020电压控制版本, 仅保留电流控制版本; 从 dvc_motor_dji 中独立拆分
 *
 * @note M3508常规用法为速度环(OMEGA)控制, 默认控制方式为 OMEGA; 也支持 ANGLE 位置闭环双环用法
 *       C620配M3508电机(减速比3591:187); M2006/C610见 dvc_motor_dji_2006.h
 *       发送帧使用 0x1ff/0x200
 *       公共类型见 dvc_motor_dji.h
 *
 * @copyright USTC-RoboWalker (c) 2023-2025
 *
 */
#ifndef DVC_MOTOR_DJI_3508_H
#define DVC_MOTOR_DJI_3508_H

/* Includes ------------------------------------------------------------------*/

#include "dvc_motor_dji.h"

/**
 * @brief Reusable, C620无刷电调, 自带电流环, 单片机控制输出电流
 *
 */
class Class_Motor_DJI_C620
{
public:
    // PID角度环控制
    Class_PID PID_Angle;
    // PID角速度环控制
    Class_PID PID_Omega;

    void Init(const FDCAN_HandleTypeDef *hcan, const Enum_Motor_DJI_ID &__CAN_Rx_ID, const Enum_Motor_DJI_Control_Method &__Motor_DJI_Control_Method = Motor_DJI_Control_Method_OMEGA, const float &__Gearbox_Rate = 3591.0f / 187.0f);

    inline Enum_Motor_DJI_Status Get_Status() const;

    inline float Get_Now_Angle() const;

    inline float Get_Now_Omega() const;

    inline float Get_Now_Torque() const;

    inline float Get_Now_Temperature() const;

    inline float Get_Now_Power() const;

    inline Enum_Motor_DJI_Control_Method Get_Control_Method() const;

    inline float Get_Target_Angle() const;

    inline float Get_Target_Omega() const;

    inline float Get_Target_Torque() const;

    inline float Get_Feedforward_Omega() const;

    inline float Get_Feedforward_Torque() const;

    inline void Set_Control_Method(const Enum_Motor_DJI_Control_Method __Motor_DJI_Control_Method);

    inline void Set_Target_Angle(const float &__Target_Angle);

    inline void Set_Target_Omega(const float &__Target_Omega);

    inline void Set_Target_Torque(const float &__Target_Torque);

    inline void Set_Feedforward_Omega(const float &__Feedforward_Omega);

    inline void Set_Feedforward_Torque(const float &__Feedforward_Torque);

    void CAN_RxCpltCallback();

    void TIM_100ms_Alive_PeriodElapsedCallback();

    void TIM_Calculate_PeriodElapsedCallback();

protected:
    // 初始化相关变量

    // 绑定的CAN
    Struct_CAN_Manage_Object *CAN_Manage_Object;
    // 收数据绑定的CAN ID, C6系列0x201~0x208, GM系列0x205~0x20b
    Enum_Motor_DJI_ID CAN_Rx_ID;
    // 发送缓存区
    uint8_t *Tx_Data;
    // 减速比, 默认带减速箱
    float Gearbox_Rate;

    // 常量

    // 功率计算系数
    const float POWER_K_0 = 1.1532294690949845f;
    const float POWER_K_1 = 0.00003165414968530437f;
    const float POWER_K_2 = 683.7802569066574f;
    const float POWER_A = 0.8267041157447231f;

    // 一圈编码器刻度
    const uint16_t ENCODER_NUM_PER_ROUND = 8192;

    // 扭矩电流常数, 减速前
    const float CURRENT_TO_TORQUE = 0.3f / (3591.0f / 187.0f);
    // 电流到输出的转化系数
    const float CURRENT_TO_OUT = 16384.0f / 20.0f;
    // 最大输出刻度
    const float OUT_MAX = 16384.0f;

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
    Enum_Motor_DJI_Control_Method Motor_DJI_Control_Method = Motor_DJI_Control_Method_ANGLE;
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
 * @brief Reusable, M3508无刷减速电机 + C620无刷电调, 自带电流环, 单片机控制输出电流
 *
 */
class Class_Motor_DJI_M3508
{
public:
    // PID角度环控制
    Class_PID PID_Angle;
    // PID角速度环控制
    Class_PID PID_Omega;

    void Init(const FDCAN_HandleTypeDef *hcan, const Enum_Motor_DJI_ID &__CAN_Rx_ID, const Enum_Motor_DJI_Control_Method &__Motor_DJI_Control_Method = Motor_DJI_Control_Method_OMEGA, const float &__Gearbox_Rate = 3591.0f / 187.0f);

    inline Enum_Motor_DJI_Status Get_Status() const;

    inline float Get_Now_Angle() const;

    inline float Get_Now_Omega() const;

    inline float Get_Now_Torque() const;

    inline float Get_Now_Temperature() const;

    inline float Get_Now_Power() const;

    inline Enum_Motor_DJI_Control_Method Get_Control_Method() const;

    inline float Get_Target_Angle() const;

    inline float Get_Target_Omega() const;

    inline float Get_Target_Torque() const;

    inline float Get_Feedforward_Omega() const;

    inline float Get_Feedforward_Torque() const;

    inline void Set_Control_Method(const Enum_Motor_DJI_Control_Method __Motor_DJI_Control_Method);

    inline void Set_Target_Angle(const float &__Target_Angle);

    inline void Set_Target_Omega(const float &__Target_Omega);

    inline void Set_Target_Torque(const float &__Target_Torque);

    inline void Set_Feedforward_Omega(const float &__Feedforward_Omega);

    inline void Set_Feedforward_Torque(const float &__Feedforward_Torque);

    void CAN_RxCpltCallback();

    void TIM_100ms_Alive_PeriodElapsedCallback();

    void TIM_Calculate_PeriodElapsedCallback();

protected:
    // 初始化相关变量

    // 绑定的CAN
    Struct_CAN_Manage_Object *CAN_Manage_Object;
    // 收数据绑定的CAN ID, C6系列0x201~0x208, GM系列0x205~0x20b
    Enum_Motor_DJI_ID CAN_Rx_ID;
    // 发送缓存区
    uint8_t *Tx_Data;
    // 减速比, 默认带减速箱
    float Gearbox_Rate;

    // 常量

    // 功率计算系数
    const float POWER_K_0 = 1.1532294690949845f;
    const float POWER_K_1 = 0.00003165414968530437f;
    const float POWER_K_2 = 683.7802569066574f;
    const float POWER_A = 0.8267041157447231f;

    // 一圈编码器刻度
    const uint16_t ENCODER_NUM_PER_ROUND = 8192;

    // 扭矩电流常数, 减速前
    const float CURRENT_TO_TORQUE = 0.3f / (3591.0f / 187.0f);
    // 电流到输出的转化系数
    const float CURRENT_TO_OUT = 16384.0f / 20.0f;
    // 最大输出刻度
    const float OUT_MAX = 16384.0f;

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
    Enum_Motor_DJI_Control_Method Motor_DJI_Control_Method = Motor_DJI_Control_Method_ANGLE;
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

/* Exported variables --------------------------------------------------------*/

/* Exported function declarations --------------------------------------------*/

/**
 * @brief 获取电机状态
 *
 * @return Enum_Motor_DJI_Status 电机状态
 */
inline Enum_Motor_DJI_Status Class_Motor_DJI_C620::Get_Status() const
{
    return (Motor_DJI_Status);
}

/**
 * @brief 获取当前的角度, rad
 *
 * @return float 当前的角度, rad
 */
inline float Class_Motor_DJI_C620::Get_Now_Angle() const
{
    return (Rx_Data.Now_Angle);
}

/**
 * @brief 获取当前的速度, rad/s
 *
 * @return float 当前的速度, rad/s
 */
inline float Class_Motor_DJI_C620::Get_Now_Omega() const
{
    return (Rx_Data.Now_Omega);
}

/**
 * @brief 获取当前的扭矩, Nm
 *
 * @return float 当前的扭矩, Nm
 */
inline float Class_Motor_DJI_C620::Get_Now_Torque() const
{
    return (Rx_Data.Now_Torque);
}

/**
 * @brief 获取当前的温度, K
 *
 * @return float 当前的温度, K
 */
inline float Class_Motor_DJI_C620::Get_Now_Temperature() const
{
    return (Rx_Data.Now_Temperature);
}

/**
 * @brief 获取当前的功率, W
 *
 * @return float 当前的功率, W
 */
inline float Class_Motor_DJI_C620::Get_Now_Power() const
{
    return (Rx_Data.Now_Power);
}

/**
 * @brief 获取电机控制方式
 *
 * @return Enum_Motor_DJI_Control_Method 电机控制方式
 */
inline Enum_Motor_DJI_Control_Method Class_Motor_DJI_C620::Get_Control_Method() const
{
    return (Motor_DJI_Control_Method);
}

/**
 * @brief 获取目标的角度, rad
 *
 * @return float 目标的角度, rad
 */
inline float Class_Motor_DJI_C620::Get_Target_Angle() const
{
    return (Target_Angle);
}

/**
 * @brief 获取目标的速度, rad/s
 *
 * @return float 目标的速度, rad/s
 */
inline float Class_Motor_DJI_C620::Get_Target_Omega() const
{
    return (Target_Omega);
}

/**
 * @brief 获取目标的扭矩, Nm
 *
 * @return float 目标的扭矩, Nm
 */
inline float Class_Motor_DJI_C620::Get_Target_Torque() const
{
    return (Target_Torque);
}

/**
 * @brief 获取前馈的速度, rad/s
 *
 * @return float 前馈的速度, rad/s
 */
inline float Class_Motor_DJI_C620::Get_Feedforward_Omega() const
{
    return (Feedforward_Omega);
}

/**
 * @brief 获取前馈的扭矩, Nm
 *
 * @return float 前馈的扭矩, Nm
 */
inline float Class_Motor_DJI_C620::Get_Feedforward_Torque() const
{
    return (Feedforward_Torque);
}

/**
 * @brief 设定电机控制方式
 *
 * @param __Motor_DJI_Control_Method 电机控制方式
 */
inline void Class_Motor_DJI_C620::Set_Control_Method(Enum_Motor_DJI_Control_Method __Motor_DJI_Control_Method)
{
    Motor_DJI_Control_Method = __Motor_DJI_Control_Method;
}

/**
 * @brief 设定目标的角度, rad
 *
 * @param __Target_Angle 目标的角度, rad
 */
inline void Class_Motor_DJI_C620::Set_Target_Angle(const float &__Target_Angle)
{
    Target_Angle = __Target_Angle;
}

/**
 * @brief 设定目标的速度, rad/s
 *
 * @param __Target_Omega 目标的速度, rad/s
 */
inline void Class_Motor_DJI_C620::Set_Target_Omega(const float &__Target_Omega)
{
    Target_Omega = __Target_Omega;
}

/**
 * @brief 设定目标的扭矩, Nm
 *
 * @param __Target_Torque 目标的扭矩, Nm
 */
inline void Class_Motor_DJI_C620::Set_Target_Torque(const float &__Target_Torque)
{
    Target_Torque = __Target_Torque;
}

/**
 * @brief 设定前馈的速度, rad/s
 *
 * @param __Feedforward_Omega 前馈的速度, rad/s
 */
inline void Class_Motor_DJI_C620::Set_Feedforward_Omega(const float &__Feedforward_Omega)
{
    Feedforward_Omega = __Feedforward_Omega;
}

/**
 * @brief 设定前馈的扭矩, Nm
 *
 * @param __Feedforward_Torque 前馈的扭矩, Nm
 */
inline void Class_Motor_DJI_C620::Set_Feedforward_Torque(const float &__Feedforward_Torque)
{
    Feedforward_Torque = __Feedforward_Torque;
}
/**
 * @brief 获取电机状态
 *
 * @return Enum_Motor_DJI_Status 电机状态
 */
inline Enum_Motor_DJI_Status Class_Motor_DJI_M3508::Get_Status() const
{
    return (Motor_DJI_Status);
}

/**
 * @brief 获取当前的角度, rad
 *
 * @return float 当前的角度, rad
 */
inline float Class_Motor_DJI_M3508::Get_Now_Angle() const
{
    return (Rx_Data.Now_Angle);
}

/**
 * @brief 获取当前的速度, rad/s
 *
 * @return float 当前的速度, rad/s
 */
inline float Class_Motor_DJI_M3508::Get_Now_Omega() const
{
    return (Rx_Data.Now_Omega);
}

/**
 * @brief 获取当前的扭矩, Nm
 *
 * @return float 当前的扭矩, Nm
 */
inline float Class_Motor_DJI_M3508::Get_Now_Torque() const
{
    return (Rx_Data.Now_Torque);
}

/**
 * @brief 获取当前的温度, K
 *
 * @return float 当前的温度, K
 */
inline float Class_Motor_DJI_M3508::Get_Now_Temperature() const
{
    return (Rx_Data.Now_Temperature);
}

/**
 * @brief 获取当前的功率, W
 *
 * @return float 当前的功率, W
 */
inline float Class_Motor_DJI_M3508::Get_Now_Power() const
{
    return (Rx_Data.Now_Power);
}

/**
 * @brief 获取电机控制方式
 *
 * @return Enum_Motor_DJI_Control_Method 电机控制方式
 */
inline Enum_Motor_DJI_Control_Method Class_Motor_DJI_M3508::Get_Control_Method() const
{
    return (Motor_DJI_Control_Method);
}

/**
 * @brief 获取目标的角度, rad
 *
 * @return float 目标的角度, rad
 */
inline float Class_Motor_DJI_M3508::Get_Target_Angle() const
{
    return (Target_Angle);
}

/**
 * @brief 获取目标的速度, rad/s
 *
 * @return float 目标的速度, rad/s
 */
inline float Class_Motor_DJI_M3508::Get_Target_Omega() const
{
    return (Target_Omega);
}

/**
 * @brief 获取目标的扭矩, Nm
 *
 * @return float 目标的扭矩, Nm
 */
inline float Class_Motor_DJI_M3508::Get_Target_Torque() const
{
    return (Target_Torque);
}

/**
 * @brief 获取前馈的速度, rad/s
 *
 * @return float 前馈的速度, rad/s
 */
inline float Class_Motor_DJI_M3508::Get_Feedforward_Omega() const
{
    return (Feedforward_Omega);
}

/**
 * @brief 获取前馈的扭矩, Nm
 *
 * @return float 前馈的扭矩, Nm
 */
inline float Class_Motor_DJI_M3508::Get_Feedforward_Torque() const
{
    return (Feedforward_Torque);
}

/**
 * @brief 设定电机控制方式
 *
 * @param __Motor_DJI_Control_Method 电机控制方式
 */
inline void Class_Motor_DJI_M3508::Set_Control_Method(Enum_Motor_DJI_Control_Method __Motor_DJI_Control_Method)
{
    Motor_DJI_Control_Method = __Motor_DJI_Control_Method;
}

/**
 * @brief 设定目标的角度, rad
 *
 * @param __Target_Angle 目标的角度, rad
 */
inline void Class_Motor_DJI_M3508::Set_Target_Angle(const float &__Target_Angle)
{
    Target_Angle = __Target_Angle;
}

/**
 * @brief 设定目标的速度, rad/s
 *
 * @param __Target_Omega 目标的速度, rad/s
 */
inline void Class_Motor_DJI_M3508::Set_Target_Omega(const float &__Target_Omega)
{
    Target_Omega = __Target_Omega;
}

/**
 * @brief 设定目标的扭矩, Nm
 *
 * @param __Target_Torque 目标的扭矩, Nm
 */
inline void Class_Motor_DJI_M3508::Set_Target_Torque(const float &__Target_Torque)
{
    Target_Torque = __Target_Torque;
}

/**
 * @brief 设定前馈的速度, rad/s
 *
 * @param __Feedforward_Omega 前馈的速度, rad/s
 */
inline void Class_Motor_DJI_M3508::Set_Feedforward_Omega(const float &__Feedforward_Omega)
{
    Feedforward_Omega = __Feedforward_Omega;
}

/**
 * @brief 设定前馈的扭矩, Nm
 *
 * @param __Feedforward_Torque 前馈的扭矩, Nm
 */
inline void Class_Motor_DJI_M3508::Set_Feedforward_Torque(const float &__Feedforward_Torque)
{
    Feedforward_Torque = __Feedforward_Torque;
}

/**
 * @copyright USTC-RoboWalker (c) 2026
 */
/* ---------------------------------------------------------------------------
 * 2. M3508 速度闭环控制
 *    结构: 斜坡限制 -> 速度环PID -> 电流, 5ms 一步
 *    目标: 0.5s 后 20·sin(t) rad/s, 运行 300s 后停
 * ------------------------------------------------------------------------- */

/**
 * @brief M3508 速度闭环控制参数(默认值与 Zephyr 样例完全一致)
 *
 */
struct Struct_Motor_DJI_M3508_Speed_Control_Config
{
    // 速度环 PID, 输出 = 电流 A
    float Velocity_KP = 0.02f;
    float Velocity_KI = 0.05f;
    float Velocity_KD = 0.0f;
    // 速度环积分限幅(电流), A
    float Velocity_Integral_Max_A = 0.1f;
    // 静态前馈系数(样例 feedforward.k_static=0.005, 电流 = k_static * 速度参考)
    float Velocity_Feedforward_K_Static = 0.005f;
    // 速度参考斜坡限制, rad/s^2
    float Velocity_Ramp_RadS2 = 100.0f;
    // 速度测量低通滤波时间常数, s (样例 0.025)
    float Velocity_Filter_Tau_S = 0.025f;
    // 速度软死区, rad/s
    float Velocity_Dead_Zone_RadS = 0.20f;
    // 电流软限幅, A
    float Current_Max_A = 0.8f;
    // M3508(减速后)扭矩常数(电流->输出轴扭矩), N·m/A
    float Current_To_Torque_Nm_A = 0.3f;
    // 控制周期, s
    float Control_Period_S = 0.005f;
};

/**
 * @brief 获取 M3508 速度闭环默认参数(Zephyr 样例值)
 *
 * @return Struct_Motor_DJI_M3508_Speed_Control_Config 默认参数
 */
inline Struct_Motor_DJI_M3508_Speed_Control_Config Motor_DJI_M3508_Speed_Control_Config_Default()
{
    Struct_Motor_DJI_M3508_Speed_Control_Config tmp_config;
    return (tmp_config);
}

/**
 * @brief Reusable, M3508 速度闭环控制器
 *
 */
class Class_Motor_DJI_M3508_Speed_Control
{
public:
    // 底层电机(传感器 + 执行器)
    Class_Motor_DJI_M3508 Motor;
    // 速度环 PID
    Class_PID PID_Omega;

    // 遥测读变量
    float Velocity_Reference_RadS = 0.0f;
    float Filtered_Omega_RadS = 0.0f;
    float Now_Omega_RadS = 0.0f;
    float Target_Current_A = 0.0f;
    float Now_Torque_Nm = 0.0f;

    void Init(const FDCAN_HandleTypeDef *hcan, const Enum_Motor_DJI_ID &__CAN_Rx_ID, const float &__Gearbox_Rate = 3591.0f / 187.0f, const Struct_Motor_DJI_M3508_Speed_Control_Config &__Config = Motor_DJI_M3508_Speed_Control_Config_Default());

    // 在 1ms 周期任务中调用, 内部按 Control_Period 节拍(5ms)计算; elapsed_ms 为从启动起的运行时长(ms)
    void TIM_1ms_PeriodElapsedCallback(const int64_t &elapsed_ms);

    // 关闭内置目标序列后, 可手动下发目标速度(rad/s)
    inline void Set_Sample_Target_Enable(const bool &__Sample_Target_Enable);

    inline void Set_Target_Omega_Manually(const float &__Target_Omega_RadS);

    // 输出使能(任务层安全链路): false 时不下发任何扭矩(电机自由), 并清掉速度环积分/斜坡/滤波状态
    bool Output_Enable = true;

    // 设定/读取输出使能
    inline void Set_Output_Enable(const bool &__Output_Enable);

    inline bool Get_Output_Enable() const;

protected:
    // 内置配置
    Struct_Motor_DJI_M3508_Speed_Control_Config Config;
    // 是否使用内置样例目标序列
    bool Sample_Target_Enable = true;
    // 手动目标速度, rad/s
    float Target_Omega_Manually = 0.0f;
    // 前一拍速度参考(斜坡限制状态), rad/s
    float Pre_Velocity_Reference = 0.0f;
    // 滤波速度(低通滤波状态), rad/s
    float Filtered_Omega = 0.0f;
    // 滤波初始化标志(首拍用测量值初始化, 避免启动瞬态)
    bool Filter_Initialized = false;
    // 1ms 节拍计数器
    uint32_t Cnt_1ms = 0;

    // 内置目标序列: 0.5s 后 20·sin(t) rad/s, 300s 后停止
    float Target_Omega_From_Elapsed_Time(const int64_t &elapsed_ms);
};
/**
 * @brief 获取内置样例目标速度序列值(rad/s): 0.5s 后 20·sin(t), 300s 后归零
 *
 * @param elapsed_ms 运行时长, ms
 * @return float 目标速度, rad/s
 */
inline float Class_Motor_DJI_M3508_Speed_Control::Target_Omega_From_Elapsed_Time(const int64_t &elapsed_ms)
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

/**
 * @brief 设定是否使用内置样例目标序列
 *
 * @param __Sample_Target_Enable true=内置序列  false=使用手动目标
 */
inline void Class_Motor_DJI_M3508_Speed_Control::Set_Sample_Target_Enable(const bool &__Sample_Target_Enable)
{
    Sample_Target_Enable = __Sample_Target_Enable;
}

/**
 * @brief 手动设定目标速度(需先 Set_Sample_Target_Enable(false))
 *
 * @param __Target_Omega_RadS 目标速度, rad/s
 */
inline void Class_Motor_DJI_M3508_Speed_Control::Set_Target_Omega_Manually(const float &__Target_Omega_RadS)
{
    Target_Omega_Manually = __Target_Omega_RadS;
}

/**
 * @brief 设定输出使能(任务层安全链路)
 *
 * @param __Output_Enable true=正常闭环出力  false=不下发任何扭矩(电机自由)
 */
inline void Class_Motor_DJI_M3508_Speed_Control::Set_Output_Enable(const bool &__Output_Enable)
{
    Output_Enable = __Output_Enable;
}

/**
 * @brief 获取输出使能
 *
 * @return bool 输出使能
 */
inline bool Class_Motor_DJI_M3508_Speed_Control::Get_Output_Enable() const
{
    return (Output_Enable);
}

#endif

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
