#pragma once
#include <cstddef>
#include <cstdint>
#include "1_Middleware/Algorithm/IMU/alg_gimbal_ahrs.h"

// DM-IMU-L1, manufacturer manual V1.2. CAN is 1 Mbps.
namespace dm_imu {
constexpr std::uint16_t device_id = 1, master_id = 0x11;
constexpr std::uint32_t timeout_ms = 50;
struct Snapshot {
    float yaw_rad = 0, yaw_zero_rad = 0, yaw_delta_rad = 0, yaw_rate_rad_s = 0;
    std::uint32_t sample_age_ms = 0, sequence = 0;
    std::uint16_t feedback_id = 0;
    bool valid = false, zero_valid = false;
    Vector3 raw_accel_mps2{}, corrected_accel_mps2{}, raw_gyro_rad_s{}, gyro_bias_rad_s{};
    float vendor_yaw_rad = 0, pitch_rad = 0, roll_rad = 0, yaw_accel_rad_s2 = 0;
    std::uint32_t accel_age_ms = 0;
    // Independent vendor heading path for the IMU mounted on small yaw.
    // Does not apply the legacy big-axis lever-arm AHRS to small yaw.
    float world_yaw_delta_rad = 0;
    std::uint32_t world_yaw_age_ms = 0, gyro_age_ms = 0, world_sequence = 0;
    bool world_valid = false;
};
class Sensor {
public:
    void init(std::uint32_t now);
    // Caller serializes ISR writes and task reads. IDs read back from the real IMU.
    bool consume(std::uint16_t id, const std::uint8_t *data, std::size_t size, std::uint32_t now);
    Snapshot snapshot(std::uint32_t now) const;
    std::uint32_t euler_count = 0, gyro_count = 0, rejected_count = 0;
    std::uint32_t accel_count = 0, fusion_count = 0;
private:
    void fuse(std::uint32_t now);
    float yaw_ = 0, zero_ = 0, fused_yaw_ = 0, fused_pitch_ = 0, fused_roll_ = 0;
    float last_vendor_yaw_ = 0;
    Vector3 accel_{}, gyro_{}, bias_{}, bias_sum_{};
    GimbalAhrs ahrs_{};
    std::uint32_t started_ = 0, yaw_tick_ = 0, gyro_tick_ = 0, stable_count_ = 0;
    std::uint32_t accel_tick_ = 0, fusion_tick_ = 0;
    std::uint16_t feedback_id_ = 0;
    bool id_known_ = false, zero_valid_ = false, fusion_ready_ = false;
};
// Round-robin gyro, acceleration and vendor Euler read-only requests.
std::size_t request(unsigned slot, std::uint16_t &id, std::uint8_t data[8]);
} // namespace dm_imu
