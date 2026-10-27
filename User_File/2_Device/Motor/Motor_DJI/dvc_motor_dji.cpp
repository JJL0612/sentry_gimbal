/** @file dvc_motor_dji.cpp
 * @brief DJI 公共 CAN 缓冲区分配与功率估算
 * @copyright USTC-RoboWalker (c) 2023-2025
 */
#include "dvc_motor_dji.h"

uint8_t *allocate_tx_data(const FDCAN_HandleTypeDef *hcan, const Enum_Motor_DJI_ID &__CAN_ID, const Enum_Motor_DJI_GM6020_Driver_Version &__DJI_Motor_Driver_Version)
{
    uint8_t *tmp_tx_data_ptr = nullptr;
    if (hcan == &hfdcan1)
    {
        switch (__CAN_ID)
        {
        case (Motor_DJI_ID_0x201):
        {
            tmp_tx_data_ptr = &(CAN1_0x200_Tx_Data[0]);

            break;
        }
        case (Motor_DJI_ID_0x202):
        {
            tmp_tx_data_ptr = &(CAN1_0x200_Tx_Data[2]);

            break;
        }
        case (Motor_DJI_ID_0x203):
        {
            tmp_tx_data_ptr = &(CAN1_0x200_Tx_Data[4]);

            break;
        }
        case (Motor_DJI_ID_0x204):
        {
            tmp_tx_data_ptr = &(CAN1_0x200_Tx_Data[6]);

            break;
        }
        case (Motor_DJI_ID_0x205):
        {
            if (__DJI_Motor_Driver_Version == Motor_DJI_GM6020_Driver_Version_DEFAULT)
            {
                tmp_tx_data_ptr = &(CAN1_0x1ff_Tx_Data[0]);
            }
            else
            {
                tmp_tx_data_ptr = &(CAN1_0x1fe_Tx_Data[0]);
            }

            break;
        }
        case (Motor_DJI_ID_0x206):
        {
            if (__DJI_Motor_Driver_Version == Motor_DJI_GM6020_Driver_Version_DEFAULT)
            {
                tmp_tx_data_ptr = &(CAN1_0x1ff_Tx_Data[2]);
            }
            else
            {
                tmp_tx_data_ptr = &(CAN1_0x1fe_Tx_Data[2]);
            }

            break;
        }
        case (Motor_DJI_ID_0x207):
        {
            if (__DJI_Motor_Driver_Version == Motor_DJI_GM6020_Driver_Version_DEFAULT)
            {
                tmp_tx_data_ptr = &(CAN1_0x1ff_Tx_Data[4]);
            }
            else
            {
                tmp_tx_data_ptr = &(CAN1_0x1fe_Tx_Data[4]);
            }

            break;
        }
        case (Motor_DJI_ID_0x208):
        {
            if (__DJI_Motor_Driver_Version == Motor_DJI_GM6020_Driver_Version_DEFAULT)
            {
                tmp_tx_data_ptr = &(CAN1_0x1ff_Tx_Data[6]);
            }
            else
            {
                tmp_tx_data_ptr = &(CAN1_0x1fe_Tx_Data[6]);
            }

            break;
        }
        case (Motor_DJI_ID_0x209):
        {
            if (__DJI_Motor_Driver_Version == Motor_DJI_GM6020_Driver_Version_DEFAULT)
            {
                tmp_tx_data_ptr = &(CAN1_0x2ff_Tx_Data[0]);
            }
            else
            {
                tmp_tx_data_ptr = &(CAN1_0x2fe_Tx_Data[0]);
            }

            break;
        }
        case (Motor_DJI_ID_0x20A):
        {
            if (__DJI_Motor_Driver_Version == Motor_DJI_GM6020_Driver_Version_DEFAULT)
            {
                tmp_tx_data_ptr = &(CAN1_0x2ff_Tx_Data[2]);
            }
            else
            {
                tmp_tx_data_ptr = &(CAN1_0x2fe_Tx_Data[2]);
            }

            break;
        }
        case (Motor_DJI_ID_0x20B):
        {
            if (__DJI_Motor_Driver_Version == Motor_DJI_GM6020_Driver_Version_DEFAULT)
            {
                tmp_tx_data_ptr = &(CAN1_0x2ff_Tx_Data[4]);
            }
            else
            {
                tmp_tx_data_ptr = &(CAN1_0x2fe_Tx_Data[4]);
            }

            break;
        }
        }
    }
    else if (hcan == &hfdcan2)
    {
        switch (__CAN_ID)
        {
        case (Motor_DJI_ID_0x201):
        {
            tmp_tx_data_ptr = &(CAN2_0x200_Tx_Data[0]);

            break;
        }
        case (Motor_DJI_ID_0x202):
        {
            tmp_tx_data_ptr = &(CAN2_0x200_Tx_Data[2]);

            break;
        }
        case (Motor_DJI_ID_0x203):
        {
            tmp_tx_data_ptr = &(CAN2_0x200_Tx_Data[4]);

            break;
        }
        case (Motor_DJI_ID_0x204):
        {
            tmp_tx_data_ptr = &(CAN2_0x200_Tx_Data[6]);

            break;
        }
        case (Motor_DJI_ID_0x205):
        {
            if (__DJI_Motor_Driver_Version == Motor_DJI_GM6020_Driver_Version_DEFAULT)
            {
                tmp_tx_data_ptr = &(CAN2_0x1ff_Tx_Data[0]);
            }
            else
            {
                tmp_tx_data_ptr = &(CAN2_0x1fe_Tx_Data[0]);
            }

            break;
        }
        case (Motor_DJI_ID_0x206):
        {
            if (__DJI_Motor_Driver_Version == Motor_DJI_GM6020_Driver_Version_DEFAULT)
            {
                tmp_tx_data_ptr = &(CAN2_0x1ff_Tx_Data[2]);
            }
            else
            {
                tmp_tx_data_ptr = &(CAN2_0x1fe_Tx_Data[2]);
            }

            break;
        }
        case (Motor_DJI_ID_0x207):
        {
            if (__DJI_Motor_Driver_Version == Motor_DJI_GM6020_Driver_Version_DEFAULT)
            {
                tmp_tx_data_ptr = &(CAN2_0x1ff_Tx_Data[4]);
            }
            else
            {
                tmp_tx_data_ptr = &(CAN2_0x1fe_Tx_Data[4]);
            }

            break;
        }
        case (Motor_DJI_ID_0x208):
        {
            if (__DJI_Motor_Driver_Version == Motor_DJI_GM6020_Driver_Version_DEFAULT)
            {
                tmp_tx_data_ptr = &(CAN2_0x1ff_Tx_Data[6]);
            }
            else
            {
                tmp_tx_data_ptr = &(CAN2_0x1fe_Tx_Data[6]);
            }

            break;
        }
        case (Motor_DJI_ID_0x209):
        {
            if (__DJI_Motor_Driver_Version == Motor_DJI_GM6020_Driver_Version_DEFAULT)
            {
                tmp_tx_data_ptr = &(CAN2_0x2ff_Tx_Data[0]);
            }
            else
            {
                tmp_tx_data_ptr = &(CAN2_0x2fe_Tx_Data[0]);
            }

            break;
        }
        case (Motor_DJI_ID_0x20A):
        {
            if (__DJI_Motor_Driver_Version == Motor_DJI_GM6020_Driver_Version_DEFAULT)
            {
                tmp_tx_data_ptr = &(CAN2_0x2ff_Tx_Data[2]);
            }
            else
            {
                tmp_tx_data_ptr = &(CAN2_0x2fe_Tx_Data[2]);
            }

            break;
        }
        case (Motor_DJI_ID_0x20B):
        {
            if (__DJI_Motor_Driver_Version == Motor_DJI_GM6020_Driver_Version_DEFAULT)
            {
                tmp_tx_data_ptr = &(CAN2_0x2ff_Tx_Data[4]);
            }
            else
            {
                tmp_tx_data_ptr = &(CAN2_0x2fe_Tx_Data[4]);
            }

            break;
        }
        }
    }
    else if (hcan == &hfdcan3)
    {
        switch (__CAN_ID)
        {
        case (Motor_DJI_ID_0x201):
        {
            tmp_tx_data_ptr = &(CAN3_0x200_Tx_Data[0]);

            break;
        }
        case (Motor_DJI_ID_0x202):
        {
            tmp_tx_data_ptr = &(CAN3_0x200_Tx_Data[2]);

            break;
        }
        case (Motor_DJI_ID_0x203):
        {
            tmp_tx_data_ptr = &(CAN3_0x200_Tx_Data[4]);

            break;
        }
        case (Motor_DJI_ID_0x204):
        {
            tmp_tx_data_ptr = &(CAN3_0x200_Tx_Data[6]);

            break;
        }
        case (Motor_DJI_ID_0x205):
        {
            if (__DJI_Motor_Driver_Version == Motor_DJI_GM6020_Driver_Version_DEFAULT)
            {
                tmp_tx_data_ptr = &(CAN3_0x1ff_Tx_Data[0]);
            }
            else
            {
                tmp_tx_data_ptr = &(CAN3_0x1fe_Tx_Data[0]);
            }

            break;
        }
        case (Motor_DJI_ID_0x206):
        {
            if (__DJI_Motor_Driver_Version == Motor_DJI_GM6020_Driver_Version_DEFAULT)
            {
                tmp_tx_data_ptr = &(CAN3_0x1ff_Tx_Data[2]);
            }
            else
            {
                tmp_tx_data_ptr = &(CAN3_0x1fe_Tx_Data[2]);
            }

            break;
        }
        case (Motor_DJI_ID_0x207):
        {
            if (__DJI_Motor_Driver_Version == Motor_DJI_GM6020_Driver_Version_DEFAULT)
            {
                tmp_tx_data_ptr = &(CAN3_0x1ff_Tx_Data[4]);
            }
            else
            {
                tmp_tx_data_ptr = &(CAN3_0x1fe_Tx_Data[4]);
            }
            break;
        }
        case (Motor_DJI_ID_0x208):
        {
            if (__DJI_Motor_Driver_Version == Motor_DJI_GM6020_Driver_Version_DEFAULT)
            {
                tmp_tx_data_ptr = &(CAN3_0x1ff_Tx_Data[6]);
            }
            else
            {
                tmp_tx_data_ptr = &(CAN3_0x1fe_Tx_Data[6]);
            }
            break;
        }
        case (Motor_DJI_ID_0x209):
        {
            if (__DJI_Motor_Driver_Version == Motor_DJI_GM6020_Driver_Version_DEFAULT)
            {
                tmp_tx_data_ptr = &(CAN3_0x2ff_Tx_Data[0]);
            }
            else
            {
                tmp_tx_data_ptr = &(CAN3_0x2fe_Tx_Data[0]);
            }
            break;
        }
        case (Motor_DJI_ID_0x20A):
        {
            if (__DJI_Motor_Driver_Version == Motor_DJI_GM6020_Driver_Version_DEFAULT)
            {
                tmp_tx_data_ptr = &(CAN3_0x2ff_Tx_Data[2]);
            }
            else
            {
                tmp_tx_data_ptr = &(CAN3_0x2fe_Tx_Data[2]);
            }
            break;
        }
        case (Motor_DJI_ID_0x20B):
        {
            if (__DJI_Motor_Driver_Version == Motor_DJI_GM6020_Driver_Version_DEFAULT)
            {
                tmp_tx_data_ptr = &(CAN3_0x2ff_Tx_Data[4]);
            }
            else
            {
                tmp_tx_data_ptr = &(CAN3_0x2fe_Tx_Data[4]);
            }
            break;
        }
        }
    }
    return (tmp_tx_data_ptr);
}

/**
 * @brief 估计功率值
 *
 * @param K_0 电机建模系数
 * @param K_1 电机建模系数
 * @param K_2 电机建模系数
 * @param A 电机建模系数
 * @param Torque 减速箱前扭矩, Nm
 * @param Omega 减速箱前角速度, rad/s
 * @return
 */
float power_calculate(float K_0, float K_1, float K_2, float A, float Torque, float Omega)
{
    return (K_0 * Torque * Omega + K_1 * Omega * Omega + K_2 * Torque * Torque + A);
}

