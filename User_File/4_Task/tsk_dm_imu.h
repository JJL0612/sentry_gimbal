#pragma once
#include "2_Device/IMU/dvc_dm_imu.h"
#include "2_Device/Communication/dvc_chassis_link.h"
#include "fdcan.h"
extern dm_imu::Sensor Gimbal_DM_IMU;
extern volatile std::uint32_t dm_imu_tx_ok, dm_imu_tx_failed;
void Dm_Imu_Init();
void Dm_Imu_Rx(FDCAN_RxHeaderTypeDef &header, std::uint8_t *data);
void Dm_Imu_Tick();
void Dm_Imu_FillInput(chassis_link::Input &input);
