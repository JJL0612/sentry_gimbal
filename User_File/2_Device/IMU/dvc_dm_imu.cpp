#include "dvc_dm_imu.h"
#include <cmath>
#include <cstring>

namespace dm_imu {
namespace {
constexpr float pi = 3.14159265358979323846f;
float wrap(float v) { return std::remainder(v, 2.0f * pi); }
unsigned le16(const std::uint8_t *p) { return p[0] | (unsigned(p[1]) << 8); }
}
void Sensor::init(std::uint32_t now) { *this = Sensor{}; started_ = now; }
bool Sensor::consume(std::uint16_t id, const std::uint8_t *p, std::size_t size, std::uint32_t now) {
    if (!p || size != 8 || id != master_id || (p[0] != 2 && p[0] != 3) || p[1] != 0 ||
        (id_known_ && id != feedback_id_)) { ++rejected_count; return false; }
    feedback_id_ = id; id_known_ = true;
    if (p[0] == 2) {
        rate_ = float(le16(p + 6)) * (69.76f / 65535.0f) - 34.88f;
        gyro_tick_ = now; ++gyro_count;
    } else {
        yaw_ = (float(le16(p + 4)) * (360.0f / 65535.0f) - 180.0f) * pi / 180.0f;
        const bool continuous = euler_count && std::uint32_t(now - yaw_tick_) <= timeout_ms;
        yaw_tick_ = now; ++euler_count;
        if (!zero_valid_) {
            const bool stationary = gyro_count && std::uint32_t(now - gyro_tick_) <= timeout_ms &&
                                    std::fabs(rate_) < 0.05f && std::uint32_t(now - started_) >= 1000;
            stable_count_ = stationary ? (continuous ? stable_count_ + 1 : 1) : 0;
            // Wait for twenty distinct stationary Euler samples after startup.
            if (stable_count_ >= 20) { zero_ = yaw_; zero_valid_ = true; }
        }
    }
    return true;
}
Snapshot Sensor::snapshot(std::uint32_t now) const {
    Snapshot s;
    s.yaw_rad = yaw_; s.yaw_zero_rad = zero_; s.yaw_delta_rad = wrap(yaw_ - zero_);
    s.yaw_rate_rad_s = rate_; s.sample_age_ms = std::uint32_t(now - yaw_tick_);
    s.sequence = euler_count; s.feedback_id = feedback_id_; s.zero_valid = zero_valid_;
    s.valid = zero_valid_ && s.sample_age_ms <= timeout_ms && gyro_count &&
              std::uint32_t(now - gyro_tick_) <= timeout_ms;
    return s;
}
std::size_t request(unsigned slot, std::uint16_t &id, std::uint8_t p[8]) {
    std::memset(p, 0, 8);
    const auto rid = std::uint8_t((slot & 1) ? 2 : 3);
    id = device_id; p[0] = 0xcc; p[1] = rid; p[2] = 0; p[3] = 0xdd; return 8;
}
} // namespace dm_imu
