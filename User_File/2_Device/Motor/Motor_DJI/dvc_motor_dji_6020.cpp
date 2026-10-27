/**
 * @file dvc_motor_dji_6020.cpp
 * @author yssickjgd (1345578933@qq.com)
 * @brief 大疆GM6020无刷电机驱动与位置闭环实现
 * @version 2.1
 * @date 2024-01-01 1.1 官方6020驱动更新, 适配电压控制与电流控制
 * @date 2025-12-03 2.1 彻底抛弃6020电压控制版本, 仅保留电流控制版本; 从 dvc_motor_dji 中独立拆分
 *
 * @copyright USTC-RoboWalker (c) 2023-2025
 *
 */

/* Includes ------------------------------------------------------------------*/

#include "dvc_motor_dji_6020.h"

/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function declarations ---------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/

/**
 * @brief 电机初始化
 *
 * @param hcan 绑定的CAN总线
 * @param __CAN_Rx_ID 绑定的CAN ID
 * @param __Motor_DJI_Control_Method 电机控制方式, 默认角度
 * @param __Encoder_Offset 编码器偏移, 默认0
 * @param __Nearest_Angle 就近转位的单次最大旋转角度, 其数值一般为圆周的整数倍或纯小数倍, 且纯小数倍时可均分圆周, 0表示不启用就近转位
 */
void Class_Motor_DJI_GM6020::Init(const FDCAN_HandleTypeDef *hcan, const Enum_Motor_DJI_ID &__CAN_Rx_ID, const Enum_Motor_DJI_Control_Method &__Motor_DJI_Control_Method, const int32_t &__Encoder_Offset, const float &__Nearest_Angle)
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
    Encoder_Offset = __Encoder_Offset;
    Nearest_Angle = __Nearest_Angle;
    // 重新初始化后需要重新建立首帧编码器基准
    Rx_Data_Initialized = false;
    Tx_Data = allocate_tx_data(hcan, __CAN_Rx_ID, Motor_DJI_GM6020_Driver_Version_6020);
}

/**
 * @brief CAN通信接收回调函数
 *
 */
void Class_Motor_DJI_GM6020::CAN_RxCpltCallback()
{
    // 滑动窗口, 判断电机是否在线
    Flag += 1;

    Data_Process();
}

/**
 * @brief TIM定时器中断定期检测电机是否存活
 *
 */
void Class_Motor_DJI_GM6020::TIM_100ms_Alive_PeriodElapsedCallback()
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
void Class_Motor_DJI_GM6020::TIM_Calculate_PeriodElapsedCallback()
{
    PID_Calculate();

    Out = (Target_Torque + Feedforward_Torque) / CURRENT_TO_TORQUE * CURRENT_TO_OUT;
    Basic_Math_Constrain(&Out, -OUT_MAX, OUT_MAX);

    Output();

    Feedforward_Omega = 0.0f;
    Feedforward_Torque = 0.0f;
}

/**
 * @brief 数据处理过程
 *
 */
void Class_Motor_DJI_GM6020::Data_Process()
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
    if (!Rx_Data_Initialized)
    {
        // 首帧仅建立编码器基准, 不参与圈数判定
        // (否则上电时编码器值大于 4096 会被判为反方向转过一圈, Total_Round 被误减 1, 角度整体偏移 2π)
        Rx_Data_Initialized = true;
        Rx_Data.Total_Round = 0;
        Rx_Data.Total_Encoder = tmp_encoder + Encoder_Offset;
    }
    else
    {
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
    }

    // 计算电机本身信息
    Rx_Data.Now_Angle = (float) Rx_Data.Total_Encoder / (float) ENCODER_NUM_PER_ROUND * 2.0f * PI;
    Rx_Data.Now_Omega = (float) tmp_omega * BASIC_MATH_RPM_TO_RADPS;
    Rx_Data.Now_Torque = tmp_current / CURRENT_TO_OUT * CURRENT_TO_TORQUE;
    Rx_Data.Now_Temperature = tmp_buffer->Temperature + BASIC_MATH_CELSIUS_TO_KELVIN;
    Rx_Data.Now_Power = power_calculate(POWER_K_0, POWER_K_1, POWER_K_2, POWER_A, Rx_Data.Now_Torque, Rx_Data.Now_Omega);

    // 存储预备信息
    Rx_Data.Pre_Encoder = tmp_encoder;
}

/**
 * @brief 计算PID
 *
 */
void Class_Motor_DJI_GM6020::PID_Calculate()
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
void Class_Motor_DJI_GM6020::Output()
{
    Tx_Data[0] = (int16_t) Out >> 8;
    Tx_Data[1] = (int16_t) Out;
}

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/

/**
 * @copyright USTC-RoboWalker (c) 2026
 */
/* ===========================================================================
 * 1. GM6020 位置闭环控制
 * =========================================================================== */

/**
 * @brief 初始化 GM6020 位置闭环控制器
 *
 * @param hcan 绑定的CAN总线
 * @param __CAN_Rx_ID 绑定的CAN ID
 * @param __Encoder_Offset 编码器偏移, 默认0
 * @param __Config 控制参数(默认与 Zephyr 样例一致)
 */
void Class_Motor_DJI_GM6020_Position_Control::Init(const FDCAN_HandleTypeDef *hcan, const Enum_Motor_DJI_ID &__CAN_Rx_ID, const int32_t &__Encoder_Offset, const Struct_Motor_DJI_GM6020_Position_Control_Config &__Config)
{
    Config = __Config;

    // 电机作为传感器+执行器: 控制方式 TORQUE(电机类只做电流输出与 CAN 收发)
    Motor.Init(hcan, __CAN_Rx_ID, Motor_DJI_Control_Method_TORQUE, __Encoder_Offset, 0.0f);

    // 位置环 PID: 输出 = 速度参考(rad/s), 死区/积分/输出限幅按样例
    // 微分先行(PID_D_First_ENABLE): d = -K_D·dθ/dt, 与 Zephyr lib/control/pid.c 的测量值微分一致,
    // 避免目标角阶跃瞬间产生 K_D·Δθ/D_T 的微分冲击使输出直接冲到限幅
    PID_Angle.Init(Config.Position_KP, Config.Position_KI, Config.Position_KD, 0.0f,
                   Config.Position_Integral_Max,     // 积分限幅
                   Config.Position_Out_Max_RadS,     // 输出限幅(速度参考上限)
                   Config.Control_Period_S,          // D_T
                   Config.Position_Dead_Zone_Rad,    // 位置死区
                   0.0f, 0.0f, 0.0f,                 // 变速积分/积分分离不启用
                   PID_D_First_ENABLE);              // 微分先行

    // 速度环 PID: 输出 = 电流(A), 积分与输出限幅均为电流软限
    PID_Omega.Init(Config.Velocity_KP, Config.Velocity_KI, Config.Velocity_KD, 0.0f,
                   Config.Current_Max_A,             // 积分限幅(电流)
                   Config.Current_Max_A,             // 输出限幅(电流)
                   Config.Control_Period_S,
                   0.0f,                             // 速度环无死区
                   0.0f, 0.0f, 0.0f,                 // 变速积分/积分分离不启用
                   PID_D_First_ENABLE);              // 微分先行

    // 状态清零
    Cnt_1ms = 0;
    Pre_Velocity_Reference = 0.0f;
    Pid_Primed = false;
    Sample_Target_Enable = true;
    Target_Angle_Manually = 0.0f;
    Output_Enable = true;
    Manually_Target_Received = false;
    Target_Angle_Rad = 0.0f;
    Now_Angle_Rad = 0.0f;
    Velocity_Reference_RadS = 0.0f;
    Now_Omega_RadS = 0.0f;
    Target_Current_A = 0.0f;
    Now_Torque_Nm = 0.0f;
    Position_Error_Rad = 0.0f;
    Velocity_Error_RadS = 0.0f;
    Limit_Gate_Blocked = false;
    Window_Base_Rad = 0.0f;
}

/**
 * @brief 1ms 周期回调, 内部按 Control_Period(5ms)节拍执行位置闭环
 *
 * @param elapsed_ms 从启动起的运行时长, ms
 */
void Class_Motor_DJI_GM6020_Position_Control::TIM_1ms_PeriodElapsedCallback(const int64_t &elapsed_ms)
{
    Cnt_1ms++;
    if (Cnt_1ms < (uint32_t) (Config.Control_Period_S * 1000.0f))
    {
        return;
    }
    Cnt_1ms = 0;
    const float dt = Config.Control_Period_S;
    const float now_angle = Motor.Get_Now_Angle();

    // 机械限位窗口坐标换算(与累计圈数无关)
    //   window_offset: 当前角相对窗口起点的偏移(0 = 窗口起点, limit_width = 窗口终点)
    //   window_start:  当前所在这一圈的窗口起点绝对角
    //     - 绝对角模式(false): 偏移 = now_angle - Position_Min_Rad, 窗口只在第一圈, 偏移为负/超宽即限位外
    //     - 每圈周期模式(true): 偏移 = (now_angle - Position_Min_Rad) mod 2π, 每圈同一编码器区间都算窗口
    const bool limit_mode = (Config.Position_Max_Rad > Config.Position_Min_Rad);
    const float limit_width = Config.Position_Max_Rad - Config.Position_Min_Rad;
    const float two_pi = 2.0f * MOTOR_DJI_CTRL_PI;
    float window_offset;
    float window_start;
    if (limit_mode && Config.Limit_Periodic_Per_Revolution)
    {
        window_offset = fmodf(now_angle - Config.Position_Min_Rad, two_pi);
        if (window_offset < 0.0f)
        {
            window_offset += two_pi;
        }
        window_start = now_angle - window_offset;
    }
    else
    {
        window_offset = now_angle - Config.Position_Min_Rad;
        window_start = Config.Position_Min_Rad;
    }
    // 遥测基数 = 当前圈窗口起点相对第一圈窗口起点的偏移(第 k 圈 = k·2π):
    // 遥测(目标角/当前角)统一减去它, 折算到"窗口坐标"即与 tarA 同一坐标系,
    // 于是第 2、3… 圈上同一位置的显示值与第 1 圈完全一致
    const float telemetry_base = window_start - Config.Position_Min_Rad;
    Window_Base_Rad = telemetry_base;

    // 0. 反馈有效性检查: 电机离线(100ms 内没有收到反馈帧)时不输出电流,
    //    避免用上电初期的 0 角度或过期角度做闭环而乱驱动
    if (Motor.Get_Status() != Motor_DJI_Status_ENABLE)
    {
        Motor.Set_Target_Torque(0.0f);
        Motor.TIM_Calculate_PeriodElapsedCallback();

        PID_Angle.Set_Integral_Error(0.0f);
        PID_Omega.Set_Integral_Error(0.0f);
        Pre_Velocity_Reference = 0.0f;
        Pid_Primed = false;

        Now_Angle_Rad = now_angle - telemetry_base;
        Now_Omega_RadS = Motor.Get_Now_Omega();
        Now_Torque_Nm = Motor.Get_Now_Torque();
        Position_Error_Rad = 0.0f;
        Velocity_Error_RadS = 0.0f;
        return;
    }

    // 1. 目标角度(内置序列 或 手动目标)
    //    手动目标在收到上位机(VOFA)下发前跟随当前角度, 避免上电瞬间冲向机械限位
    float target_raw;
    if (Sample_Target_Enable)
    {
        target_raw = Target_Angle_From_Elapsed_Time(elapsed_ms);
    }
    else if (Manually_Target_Received)
    {
        target_raw = Target_Angle_Manually;
    }
    else
    {
        target_raw = now_angle;
    }

    // 目标角:
    //    无限位模式: 规约到当前位置的 ±π 邻域(对应样例 AbsoluteNearest 固定零点最短路径)
    //    绝对角模式: 直接软限幅到 [Position_Min_Rad, Position_Max_Rad](不做 ±π 规约, 否则目标可能被规约到限位外)
    //    每圈周期模式: 先限幅, 再映射到"当前这一圈"的窗口上(目标永远落在当前圈的窗口内)
    float target;
    if (!limit_mode)
    {
        target = now_angle + Basic_Math_Modulus_Normalization(target_raw - now_angle, two_pi);
    }
    else
    {
        const float target_offset = Basic_Math_Constrain(target_raw, Config.Position_Min_Rad, Config.Position_Max_Rad) - Config.Position_Min_Rad;
        target = window_start + target_offset;
    }

    // 1.1 机械限位门控: 当前角在机械限位窗口外时不下发任何扭矩(电机完全自由), 并清掉闭环状态;
    //     只有当前角(重新)进入限位窗口内, 才会闭环向目标角转动
    bool limit_gate_blocked = false;
    if (Config.Limit_Gate_Enable && limit_mode)
    {
        if (Config.Limit_Periodic_Per_Revolution)
        {
            // 周期模式: 偏移落在窗口内, 或正好在窗口起点之前一点(2π 附近, 由容差覆盖)都算窗口内
            limit_gate_blocked = !(window_offset <= limit_width + Config.Limit_Gate_Tolerance_Rad ||
                                   window_offset >= two_pi - Config.Limit_Gate_Tolerance_Rad);
        }
        else
        {
            limit_gate_blocked = (window_offset < -Config.Limit_Gate_Tolerance_Rad ||
                                  window_offset > limit_width + Config.Limit_Gate_Tolerance_Rad);
        }
    }

    if (limit_gate_blocked || !Output_Enable)
    {
        Limit_Gate_Blocked = limit_gate_blocked;

        Motor.Set_Target_Torque(0.0f);
        Motor.TIM_Calculate_PeriodElapsedCallback();

        PID_Angle.Set_Integral_Error(0.0f);
        PID_Omega.Set_Integral_Error(0.0f);
        Pre_Velocity_Reference = 0.0f;
        // 回到限位内后重新用当时的测量值同步 PID 历史, 避免残留的积分/微分造成跳动
        Pid_Primed = false;

        Target_Angle_Rad = target - telemetry_base;
        Now_Angle_Rad = now_angle - telemetry_base;
        Velocity_Reference_RadS = 0.0f;
        Now_Omega_RadS = Motor.Get_Now_Omega();
        Target_Current_A = 0.0f;
        Now_Torque_Nm = Motor.Get_Now_Torque();
        Position_Error_Rad = target - now_angle;
        Velocity_Error_RadS = 0.0f;
        return;
    }
    Limit_Gate_Blocked = false;

    // 1.2 首个有效控制拍: 用当前测量值同步一次 PID 历史(微分先行需要上一拍测量值),
    //     等价于 Zephyr 侧的 control_pid_reset(state, measurement), 避免启动首拍出现 D 项冲击
    if (!Pid_Primed)
    {
        Pid_Primed = true;
        PID_Angle.Set_Target(now_angle);
        PID_Angle.Set_Now(now_angle);
        PID_Angle.TIM_Calculate_PeriodElapsedCallback();
        PID_Omega.Set_Target(0.0f);
        PID_Omega.Set_Now(Motor.Get_Now_Omega());
        PID_Omega.TIM_Calculate_PeriodElapsedCallback();
        Pre_Velocity_Reference = 0.0f;
    }

    // 2. 位置环 PID -> 速度参考(rad/s)
    const float position_error = target - now_angle;
    const float position_integral_pre = PID_Angle.Get_Integral_Error();
    PID_Angle.Set_Target(target);
    PID_Angle.Set_Now(now_angle);
    PID_Angle.TIM_Calculate_PeriodElapsedCallback();
    float velocity_ref = PID_Angle.Get_Out();

    // 2.1 抗积分饱和(对齐 Zephyr control_pid_step 的 pushes_high/low_saturation):
    //     输出已饱和且误差仍在把输出推向饱和方向时, 撤销本拍积分
    if (Config.Anti_Windup_Enable && Config.Position_Out_Max_RadS > 0.0f)
    {
        if ((velocity_ref >= Config.Position_Out_Max_RadS && position_error > 0.0f) ||
            (velocity_ref <= -Config.Position_Out_Max_RadS && position_error < 0.0f))
        {
            PID_Angle.Set_Integral_Error(position_integral_pre);
        }
    }

    // 3. 速度参考斜坡限制(样例 kVelocityRampRateRadS2 = 40 rad/s^2)
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

    // 3.1 减速距离限速: |速度参考| ≤ sqrt(2·a·|位置误差|)
    //     刻意放在斜坡之后, 使"最终下发并记忆"的速度参考一定满足剩余行程内刹得住;
    //     行程仅 1.3576 rad 且两端为硬限位, 不限制就会冲进限位顶死
    if (Config.Brake_Distance_Limit_Enable && Config.Velocity_Ramp_RadS2 > 0.0f)
    {
        const float brake_velocity_max = sqrtf(2.0f * Config.Velocity_Ramp_RadS2 * Basic_Math_Abs(position_error));
        Basic_Math_Constrain(&velocity_ref, -brake_velocity_max, brake_velocity_max);
    }

    // 4. 速度环 PID -> 电流(A)
    const float now_omega = Motor.Get_Now_Omega();
    const float velocity_error = velocity_ref - now_omega;
    const float velocity_integral_pre = PID_Omega.Get_Integral_Error();
    PID_Omega.Set_Target(velocity_ref);
    PID_Omega.Set_Now(now_omega);
    PID_Omega.TIM_Calculate_PeriodElapsedCallback();
    const float current = PID_Omega.Get_Out();

    // 4.1 抗积分饱和: 电流已到软限且速度误差仍推向饱和方向时, 撤销本拍积分
    //     否则内环积分会一直积到 ±Current_Max_A, 关节到达目标后仍持续满电流
    if (Config.Anti_Windup_Enable && Config.Current_Max_A > 0.0f)
    {
        if ((current >= Config.Current_Max_A && velocity_error > 0.0f) ||
            (current <= -Config.Current_Max_A && velocity_error < 0.0f))
        {
            PID_Omega.Set_Integral_Error(velocity_integral_pre);
        }
    }

    // 5. 电流 -> 扭矩 -> 电机输出电流刻度(Torque_Sign: 电机反装时取反)
    Motor.Set_Target_Torque(Config.Torque_Sign * current * Config.Current_To_Torque_Nm_A);
    Motor.TIM_Calculate_PeriodElapsedCallback();

    // 6. 遥测(目标角/当前角统一折算到窗口坐标, 与 tarA 同坐标系, 各圈显示一致)
    Target_Angle_Rad = target - telemetry_base;
    Now_Angle_Rad = now_angle - telemetry_base;
    Velocity_Reference_RadS = velocity_ref;
    Now_Omega_RadS = now_omega;
    Target_Current_A = current;
    Now_Torque_Nm = Motor.Get_Now_Torque();
    Position_Error_Rad = position_error;
    Velocity_Error_RadS = velocity_error;
    Pre_Velocity_Reference = velocity_ref;
}
