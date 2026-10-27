/**
 * @file dvc_motor_dji.h
 * @author yssickjgd (1345578933@qq.com)
 * @brief 大疆电机公共定义(枚举/数据结构/辅助函数), 供 GM6020、M3508 与 M2006 模块共用
 * @version 2.1
 * @date 2023-08-29 0.1 23赛季定稿
 * @date 2024-01-01 1.1 官方6020驱动更新, 适配电压控制与电流控制
 * @date 2024-03-07 1.2 新增功率控制接口与相关函数
 * @date 2025-12-03 2.1 彻底抛弃6020电压控制版本, 仅保留电流控制版本; 按电机型号拆分模块:
 *                      dvc_motor_dji_6020: GM6020电机, 速度环+位置环, 位置闭环控制
 *                      dvc_motor_dji_3508: M3508/C620驱动与速度闭环
 *                      dvc_motor_dji_2006: M2006/C610驱动与速度闭环
 *
 * @copyright USTC-RoboWalker (c) 2023-2025
 *
 */
#ifndef DVC_MOTOR_DJI_H
#define DVC_MOTOR_DJI_H

/* Includes ------------------------------------------------------------------*/

#include "1_Middleware/Algorithm/PID/alg_pid.h"
#include "1_Middleware/Driver/CAN/drv_can.h"
#include <math.h>
#include <stdint.h>

#define MOTOR_DJI_CTRL_PI (3.14159265358979323846f)

/* Exported macros -----------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/

/**
 * @brief 大疆状态
 *
 */
enum Enum_Motor_DJI_Status
{
    Motor_DJI_Status_DISABLE = 0,
    Motor_DJI_Status_ENABLE,
};

/**
 * @brief 大疆电机的ID枚举类型
 *
 */
enum Enum_Motor_DJI_ID
{
    Motor_DJI_ID_0x201 = 1,
    Motor_DJI_ID_0x202,
    Motor_DJI_ID_0x203,
    Motor_DJI_ID_0x204,
    Motor_DJI_ID_0x205,
    Motor_DJI_ID_0x206,
    Motor_DJI_ID_0x207,
    Motor_DJI_ID_0x208,
    Motor_DJI_ID_0x209,
    Motor_DJI_ID_0x20A,
    Motor_DJI_ID_0x20B,
};

/**
 * @brief 大疆电机控制方式
 *
 */
enum Enum_Motor_DJI_Control_Method
{
    Motor_DJI_Control_Method_TORQUE = 0,
    Motor_DJI_Control_Method_OMEGA,
    Motor_DJI_Control_Method_ANGLE,
};

/**
 * @brief 大疆电机驱动版本, 分配CAN_ID时使用, 一般都是default, 6020电机使用
 *
 */
enum Enum_Motor_DJI_GM6020_Driver_Version
{
    Motor_DJI_GM6020_Driver_Version_DEFAULT = 0,
    Motor_DJI_GM6020_Driver_Version_6020,
};

/**
 * @brief 是否开启功率控制, 此时电机须电流作为输出模式, 不可电压控制
 *
 */
enum Enum_Motor_DJI_Power_Limit_Status
{
    Motor_DJI_Power_Limit_Status_DISABLE = 0,
    Motor_DJI_Power_Limit_Status_ENABLE,
};

/**
 * @brief 大疆电机源数据
 *
 */
struct Struct_Motor_DJI_CAN_Rx_Data
{
    uint16_t Encoder_Reverse;
    int16_t Omega_Reverse;
    int16_t Current_Reverse;
    uint8_t Temperature;
    uint8_t Reserved;
} __attribute__((packed));

/**
 * @brief 大疆电机经过处理的数据
 *
 */
struct Struct_Motor_DJI_Rx_Data
{
    float Now_Angle;
    float Now_Omega;
    float Now_Torque;
    float Now_Temperature;
    float Now_Power;
    uint32_t Pre_Encoder;
    int32_t Total_Encoder;
    int32_t Total_Round;
};

/* Exported function declarations --------------------------------------------*/

uint8_t *allocate_tx_data(const FDCAN_HandleTypeDef *hcan, const Enum_Motor_DJI_ID &__CAN_ID, const Enum_Motor_DJI_GM6020_Driver_Version &__DJI_Motor_Driver_Version = Motor_DJI_GM6020_Driver_Version_DEFAULT);
float power_calculate(float K_0, float K_1, float K_2, float A, float Torque, float Omega);

#endif

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
