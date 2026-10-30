/**
 * @file dvc_motor_dji_6020.h
 * @author yssickjgd (1345578933@qq.com)
 * @brief 大疆GM6020无刷电机配置与操作, 速度环+位置环实现位置闭环(电流控制版本)
 * @version 2.1
 * @date 2024-01-01 1.1 官方6020驱动更新, 适配电压控制与电流控制
 * @date 2025-12-03 2.1 彻底抛弃6020电压控制版本, 仅保留电流控制版本; 从 dvc_motor_dji 中独立拆分
 *
 * @note 闭环结构: 位置环PID(PID_Angle) -> 速度环PID(PID_Omega) -> 目标扭矩, 默认 ANGLE 位置闭环
 *       发送帧使用 0x1fe/0x2fe (6020官方电流驱动版本)
 *       公共类型见 dvc_motor_dji.h
 *
 * @copyright USTC-RoboWalker (c) 2023-2025
 *
 */
#ifndef DVC_MOTOR_DJI_6020_H
#define DVC_MOTOR_DJI_6020_H

/* Includes ------------------------------------------------------------------*/

#include "dvc_motor_dji.h"

/**
 * @brief Reusable, GM6020无刷电机, 单片机控制输出电压, 新驱动支持电流
 *
 */
class Class_Motor_DJI_GM6020
{
public:
    // PID角度环控制
    Class_PID PID_Angle;
    // PID角速度环控制
    Class_PID PID_Omega;

    void Init(const FDCAN_HandleTypeDef *hcan, const Enum_Motor_DJI_ID &__CAN_Rx_ID, const Enum_Motor_DJI_Control_Method &__Motor_DJI_Control_Method = Motor_DJI_Control_Method_ANGLE, const int32_t &__Encoder_Offset = 0, const float &__Nearest_Angle = 0.0f);

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
    // 初始化相关变量

    // 绑定的CAN
    Struct_CAN_Manage_Object *CAN_Manage_Object;
    // 收数据绑定的CAN ID, C6系列0x201~0x208, GM系列0x205~0x20b
    Enum_Motor_DJI_ID CAN_Rx_ID;
    // 发送缓存区
    uint8_t *Tx_Data;
    // 编码器偏移
    int32_t Encoder_Offset;
    // 就近转位的单次最大旋转角度, 其数值一般为圆周的整数倍或纯小数倍, 且纯小数倍时可均分圆周, 0表示不启用就近转位
    float Nearest_Angle;

    // 常量

    // 功率计算系数
    const float POWER_K_0 = 0.5807780373237382f;
    const float POWER_K_1 = 0.000495794444214091f;
    const float POWER_K_2 = 27.17379907809995f;
    const float POWER_A = 0.9334894382537696f;

    // 一圈编码器刻度
    const uint16_t ENCODER_NUM_PER_ROUND = 8192;

    // 扭矩电流常数
    const float CURRENT_TO_TORQUE = 0.741f;
    // 扭矩电流到输出刻度的转化系数
    const float CURRENT_TO_OUT = 16384.0f / 3.0f;
    // 最大输出刻度
    const float OUT_MAX = 16384.0f;

    // 内部变量

    // 当前时刻的电机接收flag
    uint32_t Flag = 0;
    // 前一时刻的电机接收flag
    uint32_t Pre_Flag = 0;
    // 输出量
    float Out = 0.0f;
    // 接收数据是否已建立首帧基准, 用于避免上电首个编码器值被误判为一圈
    bool Rx_Data_Initialized = false;

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
inline Enum_Motor_DJI_Status Class_Motor_DJI_GM6020::Get_Status() const
{
    return (Motor_DJI_Status);
}

/**
 * @brief 获取当前的角度, rad
 *
 * @return float 当前的角度, rad
 */
inline float Class_Motor_DJI_GM6020::Get_Now_Angle() const
{
    return (Rx_Data.Now_Angle);
}

/**
 * @brief 获取当前的速度, rad/s
 *
 * @return float 当前的速度, rad/s
 */
inline float Class_Motor_DJI_GM6020::Get_Now_Omega() const
{
    return (Rx_Data.Now_Omega);
}

/**
 * @brief 获取当前的扭矩, Nm
 *
 * @return float 当前的扭矩, Nm
 */
inline float Class_Motor_DJI_GM6020::Get_Now_Torque() const
{
    return (Rx_Data.Now_Torque);
}

/**
 * @brief 获取当前的温度, K
 *
 * @return float 当前的温度, K
 */
inline float Class_Motor_DJI_GM6020::Get_Now_Temperature() const
{
    return (Rx_Data.Now_Temperature);
}

/**
 * @brief 获取当前的功率, W
 *
 * @return float 当前的功率, W
 */
inline float Class_Motor_DJI_GM6020::Get_Now_Power() const
{
    return (Rx_Data.Now_Power);
}

/**
 * @brief 获取电机控制方式
 *
 * @return Enum_Motor_DJI_Control_Method 电机控制方式
 */
inline Enum_Motor_DJI_Control_Method Class_Motor_DJI_GM6020::Get_Control_Method() const
{
    return (Motor_DJI_Control_Method);
}

/**
 * @brief 获取目标的角度, rad
 *
 * @return float 目标的角度, rad
 */
inline float Class_Motor_DJI_GM6020::Get_Target_Angle() const
{
    return (Target_Angle);
}

/**
 * @brief 获取目标的速度, rad/s
 *
 * @return float 目标的速度, rad/s
 */
inline float Class_Motor_DJI_GM6020::Get_Target_Omega() const
{
    return (Target_Omega);
}

/**
 * @brief 获取目标的扭矩, Nm
 *
 * @return float 目标的扭矩, Nm
 */
inline float Class_Motor_DJI_GM6020::Get_Target_Torque() const
{
    return (Target_Torque);
}

/**
 * @brief 获取前馈的速度, rad/s
 *
 * @return float 前馈的速度, rad/s
 */
inline float Class_Motor_DJI_GM6020::Get_Feedforward_Omega() const
{
    return (Feedforward_Omega);
}

/**
 * @brief 获取前馈的扭矩, Nm
 *
 * @return float 前馈的扭矩, Nm
 */
inline float Class_Motor_DJI_GM6020::Get_Feedforward_Torque() const
{
    return (Feedforward_Torque);
}

/**
 * @brief 设定电机控制方式
 *
 * @param __Motor_DJI_Control_Method 电机控制方式
 */
inline void Class_Motor_DJI_GM6020::Set_Control_Method(const Enum_Motor_DJI_Control_Method &__Motor_DJI_Control_Method)
{
    Motor_DJI_Control_Method = __Motor_DJI_Control_Method;
}

/**
 * @brief 设定目标的角度, rad
 *
 * @param __Target_Angle 目标的角度, rad
 */
inline void Class_Motor_DJI_GM6020::Set_Target_Angle(const float &__Target_Angle)
{
    Target_Angle = __Target_Angle;
    if (Nearest_Angle != 0.0f)
    {
        float delta_angle = Basic_Math_Modulus_Normalization(Target_Angle - Rx_Data.Now_Angle, Nearest_Angle * 2.0f);

        Target_Angle = Rx_Data.Now_Angle + delta_angle;
    }
}

/**
 * @brief 设定目标的速度, rad/s
 *
 * @param __Target_Omega 目标的速度, rad/s
 */
inline void Class_Motor_DJI_GM6020::Set_Target_Omega(const float &__Target_Omega)
{
    Target_Omega = __Target_Omega;
}

/**
 * @brief 设定目标的扭矩, Nm
 *
 * @param __Target_Torque 目标的扭矩, Nm
 */
inline void Class_Motor_DJI_GM6020::Set_Target_Torque(const float &__Target_Torque)
{
    Target_Torque = __Target_Torque;
}

/**
 * @brief 设定前馈的速度, rad/s
 *
 * @param __Feedforward_Omega 前馈的速度, rad/s
 */
inline void Class_Motor_DJI_GM6020::Set_Feedforward_Omega(const float &__Feedforward_Omega)
{
    Feedforward_Omega = __Feedforward_Omega;
}

/**
 * @brief 设定前馈的扭矩, Nm
 *
 * @param __Feedforward_Torque 前馈的扭矩, Nm
 */
inline void Class_Motor_DJI_GM6020::Set_Feedforward_Torque(const float &__Feedforward_Torque)
{
    Feedforward_Torque = __Feedforward_Torque;
}

/**
 * @copyright USTC-RoboWalker (c) 2026
 */
/* ---------------------------------------------------------------------------
 * 1. GM6020 位置闭环控制
 *    结构: 位置环PID -> 斜坡限制 -> 速度环PID -> 电流, 5ms 一步
 *    目标: 每 5s 在 0/90/180/270 度间切换(固定零点最短路径)
 * ------------------------------------------------------------------------- */

/**
 * @brief GM6020 位置闭环控制参数(默认值与 Zephyr 样例完全一致)
 *
 */
struct Struct_Motor_DJI_GM6020_Position_Control_Config
{
    // 位置环 PID, 输出 = 速度参考 rad/s
    float Position_KP = 20.0f;
    float Position_KI = 0.5f;
    float Position_KD = 1.48f;
    // 位置环积分限幅
    float Position_Integral_Max = 8.0f;
    // 位置环输出(速度参考)上限, rad/s
    float Position_Out_Max_RadS = 30.0f;
    // 位置死区, rad
    float Position_Dead_Zone_Rad = 0.012f;
    // 速度环 PID, 输出 = 电流 A
    float Velocity_KP = 0.43f;
    float Velocity_KI = 0.55f;
    float Velocity_KD = 0.00005f;
    // 速度参考斜坡限制, rad/s^2
    float Velocity_Ramp_RadS2 = 40.0f;
    // 电流软限幅, A
    float Current_Max_A = 0.8f;
    // GM6020 电机扭矩常数(电流->扭矩, 电机轴), N·m/A
    float Current_To_Torque_Nm_A = 0.741f;
    // 控制周期, s
    float Control_Period_S = 0.005f;
    // 机械限位下限, rad (Position_Max_Rad > Position_Min_Rad 时启用软限位, 否则沿用 ±π 最短路径规约)
    float Position_Min_Rad = 0.0f;
    // 机械限位上限, rad
    float Position_Max_Rad = 0.0f;

    // ---- 以下为本工程针对"行程短 + 两端硬限位"机构的补充项(Zephyr 样例无对应字段) ----

    // 抗积分饱和: 输出已饱和且误差仍在把输出推向饱和方向时, 本拍不累积积分
    // (对齐 Zephyr lib/control/pid.c 中 pushes_high/low_saturation 判定;
    //  本工程 Class_PID 无此功能, 由封装在每次计算后撤销该拍积分实现)
    // 必须开启: 否则位置环积分会积满 ±Position_Integral_Max, 速度环积分会积满 ±Current_Max_A,
    //           关节到达目标后仍持续满电流, 只能一路冲进机械限位
    bool Anti_Windup_Enable = true;
    // 减速距离限速: |速度参考| ≤ sqrt(2 · Velocity_Ramp_RadS2 · |位置误差|)
    // 即按"剩余行程内必须能刹停"限制速度参考, 避免短行程机构冲进限位
    bool Brake_Distance_Limit_Enable = true;
    // 扭矩输出方向: +1 = 电机正电流方向与编码器角度增大方向一致(默认);
    // -1 = 相反(电机反装/编码器反向), 仅对下发电流取反, 目标角↔角度的映射(编码器零点/限位)无需改动
    float Torque_Sign = 1.0f;

    // 机械限位门控: true 时只有"当前角落在机械限位 [Position_Min_Rad, Position_Max_Rad] 内"才闭环出力;
    // 当前角在限位外(装配/上电时机构在限位外, 或被外力推出限位)时, 电机不下发任何扭矩(完全自由),
    // 需要人工/外力把它送回限位内, 进入限位后才会自动向目标角闭环转动。
    // 仅在机械限位模式(Position_Max_Rad > Position_Min_Rad)下生效。
    bool Limit_Gate_Enable = true;
    // 限位门控边界容差, rad: 判定"在限位内"时向限位外放宽的余量。
    // 用于覆盖编码器最小刻度(8192 tick/圈 → 7.7e-4 rad)、端点换算误差与静止抖动,
    // 避免机构正好停在限位端点上时门控反复通断。设为 0 即为严格按端点判定。
    float Limit_Gate_Tolerance_Rad = 0.005f;

    // 限位窗口是否"每圈重复"(周期 2π):
    //   false = 绝对角模式: 只有绝对角落在 [Position_Min_Rad, Position_Max_Rad] 这一段的才算在限位内。
    //           关节累计转过一圈(绝对角 ≥ 2π)后会被判成限位外, 即限位窗口只存在于第一圈。
    //   true  = 每圈周期模式: 把 [Position_Min_Rad, Position_Max_Rad] 看成编码器上的一段窗口,
    //           每转一圈该窗口都重新有效(第 2、3… 圈上同一编码器区间同样算在限位内);
    //           目标角也按"当前这一圈窗口内的相对偏移"映射, 即在任意一圈都能定位到窗口内的任意位置。
    bool Limit_Periodic_Per_Revolution = false;
};

/**
 * @brief 获取 GM6020 位置闭环默认参数(Zephyr 样例值)
 *
 * @return Struct_Motor_DJI_GM6020_Position_Control_Config 默认参数
 */
inline Struct_Motor_DJI_GM6020_Position_Control_Config Motor_DJI_GM6020_Position_Control_Config_Default()
{
    Struct_Motor_DJI_GM6020_Position_Control_Config tmp_config;
    return (tmp_config);
}
/**
 * @brief Reusable, GM6020 位置闭环控制器
 *
 */
class Class_Motor_DJI_GM6020_Position_Control
{
public:
    // 底层电机(传感器 + 执行器)
    Class_Motor_DJI_GM6020 Motor;
    // 位置环 PID
    Class_PID PID_Angle;
    // 速度环 PID
    Class_PID PID_Omega;

    // 遥测读变量
    float Target_Angle_Rad = 0.0f;
    float Now_Angle_Rad = 0.0f;
    float Velocity_Reference_RadS = 0.0f;
    float Now_Omega_RadS = 0.0f;
    float Target_Current_A = 0.0f;
    float Now_Torque_Nm = 0.0f;
    // 位置环误差 = 目标角 - 当前角, rad
    float Position_Error_Rad = 0.0f;
    // 速度环误差 = 速度参考 - 当前角速度, rad/s
    float Velocity_Error_RadS = 0.0f;
    // 机械限位门控是否正在锁定(当前角在机械限位外, 电机自由不出力)
    bool Limit_Gate_Blocked = false;
    // 遥测折算掉的整圈基数(第 k 圈 = k·2π): 目标角/当前角 = 绝对角 - Window_Base_Rad,
    // 便于核对"第 2、3… 圈显示值与第 1 圈一致"这一性质(该值在绝对角模式下恒为 0)
    float Window_Base_Rad = 0.0f;
    // 输出使能(任务层安全链路): false 时不下发任何扭矩(电机自由), 并清掉双环积分/斜坡/PID 历史。
    // 任务层用"遥控在线 && 使能标志"驱动本项, 即可实现遥控掉线后电机立即停止出力。
    // 与反馈离线、机械限位门控并列: 任一条件成立都不出力, 但本项由任务层显式控制
    bool Output_Enable = true;

    void Init(const FDCAN_HandleTypeDef *hcan, const Enum_Motor_DJI_ID &__CAN_Rx_ID, const int32_t &__Encoder_Offset = 0, const Struct_Motor_DJI_GM6020_Position_Control_Config &__Config = Motor_DJI_GM6020_Position_Control_Config_Default());

    // 在 1ms 周期任务中调用, 内部按 Control_Period 节拍(5ms)计算; elapsed_ms 为从启动起的运行时长(ms)
    void TIM_1ms_PeriodElapsedCallback(const int64_t &elapsed_ms);

    // 关闭内置目标序列后, 可手动下发目标角度(rad)
    inline void Set_Sample_Target_Enable(const bool &__Sample_Target_Enable);

    inline void Set_Target_Angle_Manually(const float &__Target_Angle_Rad);

    // 设定/读取输出使能(任务层安全链路用, 见 Output_Enable)
    inline void Set_Output_Enable(const bool &__Output_Enable);

    inline bool Get_Output_Enable() const;

    // Rates are in encoder-positive joint coordinates. External small-yaw world
    // gyro minus big-yaw gyro provides the measured relative rate.
    void Set_Inertial_Velocity(bool enabled, float feedforward, float measured);


protected:
    // 内置配置
    Struct_Motor_DJI_GM6020_Position_Control_Config Config;
    // 是否使用内置样例目标序列
    bool Sample_Target_Enable = true;
    // 手动目标角度, rad
    float Target_Angle_Manually = 0.0f;
    // 手动目标是否已收到(未收到时目标跟随当前角度, 避免上电瞬间冲向机械限位)
    bool Manually_Target_Received = false;
    // 上一拍速度参考(斜坡限制状态), rad/s
    float Pre_Velocity_Reference = 0.0f;
    // PID 历史是否已用当前测量值同步过(首个有效控制拍做一次, 避免微分冲击)
    bool Pid_Primed = false;
    bool Inertial_Enable = false;
    float Inertial_Feedforward = 0.0f, Inertial_Measured_Rate = 0.0f;
    // 1ms 节拍计数器
    uint32_t Cnt_1ms = 0;

    // 内置目标序列: 每 5s 在 0/90/180/270 度间切换
    float Target_Angle_From_Elapsed_Time(const int64_t &elapsed_ms);
};

/**
 * @brief 获取内置样例目标角度序列值(rad), 每 5s 切换 0/90/180/270 度
 *
 * @param elapsed_ms 运行时长, ms
 * @return float 目标角度, rad
 */
inline float Class_Motor_DJI_GM6020_Position_Control::Target_Angle_From_Elapsed_Time(const int64_t &elapsed_ms)
{
    const int64_t hold_ms = 5000;
    const int64_t phase_ms = elapsed_ms % (4 * hold_ms);
    if (phase_ms < hold_ms)
    {
        return (0.0f);
    }
    if (phase_ms < 2 * hold_ms)
    {
        return (90.0f * MOTOR_DJI_CTRL_PI / 180.0f);
    }
    if (phase_ms < 3 * hold_ms)
    {
        return (180.0f * MOTOR_DJI_CTRL_PI / 180.0f);
    }
    return (270.0f * MOTOR_DJI_CTRL_PI / 180.0f);
}

/**
 * @brief 设定是否使用内置样例目标序列
 *
 * @param __Sample_Target_Enable true=内置序列  false=使用手动目标
 */
inline void Class_Motor_DJI_GM6020_Position_Control::Set_Sample_Target_Enable(const bool &__Sample_Target_Enable)
{
    Sample_Target_Enable = __Sample_Target_Enable;
}

/**
 * @brief 手动设定目标角度(需先 Set_Sample_Target_Enable(false))
 *
 * @param __Target_Angle_Rad 目标角度, rad (仅调用本函数后才会覆盖"跟随当前角度"的初始目标)
 */
inline void Class_Motor_DJI_GM6020_Position_Control::Set_Target_Angle_Manually(const float &__Target_Angle_Rad)
{
    Target_Angle_Manually = __Target_Angle_Rad;
    Manually_Target_Received = true;
}

/**
 * @brief 设定输出使能(任务层安全链路)
 *
 * @param __Output_Enable true=正常闭环出力  false=不下发任何扭矩(电机自由)
 */
inline void Class_Motor_DJI_GM6020_Position_Control::Set_Output_Enable(const bool &__Output_Enable)
{
    Output_Enable = __Output_Enable;
}

/**
 * @brief 获取输出使能
 *
 * @return bool 输出使能
 */
inline bool Class_Motor_DJI_GM6020_Position_Control::Get_Output_Enable() const
{
    return (Output_Enable);
}

#endif

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
