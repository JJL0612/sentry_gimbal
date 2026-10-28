#pragma once
#include <cstddef>
#include <cstdint>

// DM-IMU-L1, manufacturer manual V1.1/V1.2. CAN is 1 Mbps.
namespace dm_imu {
constexpr std::uint16_t device_id = 1, master_id = 0x11;
constexpr std::uint32_t timeout_ms = 50;
struct Snapshot {
    float yaw_rad = 0, yaw_zero_rad = 0, yaw_delta_rad = 0, yaw_rate_rad_s = 0;
    std::uint32_t sample_age_ms = 0, sequence = 0;
    std::uint16_t feedback_id = 0;
    bool valid = false, zero_valid = false;
};
class Sensor {
public:
    void init(std::uint32_t now);
    // Caller serializes ISR writes and task reads. IDs read back from the real IMU.
    bool consume(std::uint16_t id, const std::uint8_t *data, std::size_t size, std::uint32_t now);
    Snapshot snapshot(std::uint32_t now) const;
    std::uint32_t euler_count = 0, gyro_count = 0, rejected_count = 0;
private:
    float yaw_ = 0, zero_ = 0, rate_ = 0;
    std::uint32_t started_ = 0, yaw_tick_ = 0, gyro_tick_ = 0, stable_count_ = 0;
    std::uint16_t feedback_id_ = 0;
    bool id_known_ = false, zero_valid_ = false;
};
// V1.2 read-only requests; real hardware responded to this format.
std::size_t request(unsigned slot, std::uint16_t &id, std::uint8_t data[8]);
} // namespace dm_imu
