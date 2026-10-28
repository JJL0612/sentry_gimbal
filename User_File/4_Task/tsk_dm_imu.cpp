#include "tsk_dm_imu.h"
#include "1_Middleware/Driver/CAN/drv_can.h"

dm_imu::Sensor Gimbal_DM_IMU;
volatile std::uint32_t dm_imu_tx_ok = 0, dm_imu_tx_failed = 0;
namespace { std::uint32_t last_poll; unsigned slot; }
void Dm_Imu_Init() { Gimbal_DM_IMU.init(HAL_GetTick()); last_poll = HAL_GetTick(); slot = 0; }
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
void Dm_Imu_FillInput(chassis_link::Input &input) {
    const auto mask = __get_PRIMASK(); __disable_irq();
    const auto sample = Gimbal_DM_IMU.snapshot(HAL_GetTick());
    __set_PRIMASK(mask);
    input.imu_present = true; input.imu_valid = sample.valid;
    input.imu_yaw_delta_rad = sample.yaw_delta_rad;
    input.imu_yaw_rate_rad_s = sample.yaw_rate_rad_s;
    input.imu_sample_age_ms = sample.sample_age_ms;
    input.imu_sequence = sample.sequence;
}
