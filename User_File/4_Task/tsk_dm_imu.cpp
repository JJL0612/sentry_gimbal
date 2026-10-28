#include "tsk_dm_imu.h"
#include "1_Middleware/Driver/CAN/drv_can.h"
#include "2_Device/BSP/BMI088/bsp_bmi088.h"

dm_imu::Sensor Gimbal_DM_IMU;
volatile std::uint32_t dm_imu_tx_ok = 0, dm_imu_tx_failed = 0;
namespace { std::uint32_t last_poll; unsigned slot; gimbal_imu::HeadingOrigin onboard_origin; }
void Dm_Imu_Init() { Gimbal_DM_IMU.init(HAL_GetTick()); last_poll = HAL_GetTick(); slot = 0; onboard_origin.init(HAL_GetTick()); }
void Dm_Imu_Rx(FDCAN_RxHeaderTypeDef &h, std::uint8_t *data) {
    if (h.IdType != FDCAN_STANDARD_ID || h.RxFrameType != FDCAN_DATA_FRAME ||
        h.DataLength != FDCAN_DLC_BYTES_8) return;
    Gimbal_DM_IMU.consume(h.Identifier, data, 8, HAL_GetTick());
}
void Dm_Imu_Tick() {
    const auto now = HAL_GetTick();
    if (std::uint32_t(now - last_poll) < 2) return;
    last_poll = now;
    std::uint8_t data[8]; std::uint16_t id;
    const auto size = dm_imu::request(slot++, id, data);
    if (CAN_Transmit_Data(&hfdcan3, id, data, size) == HAL_OK) ++dm_imu_tx_ok;
    else ++dm_imu_tx_failed;
}
// External CAN IMU polling above is diagnostic only; chassis heading uses onboard BMI088.
void Dm_Imu_FillInput(chassis_link::Input &input) {
    const auto mask = __get_PRIMASK(); __disable_irq();
    const auto raw = BSP_BMI088.Get_Heading_Sample();
    const auto now_us = SYS_Timestamp.Get_Now_Microsecond();
    __set_PRIMASK(mask);
    const auto sample = onboard_origin.update(raw,now_us,HAL_GetTick());
    input.imu_present = true; input.imu_valid = sample.valid;
    input.imu_yaw_delta_rad = sample.delta_rad;
    input.imu_yaw_rate_rad_s = sample.rate_rad_s;
    input.imu_sample_age_ms = sample.age_ms;
    input.imu_sequence = sample.sequence;
    input.imu_zero_generation = sample.generation;
}
