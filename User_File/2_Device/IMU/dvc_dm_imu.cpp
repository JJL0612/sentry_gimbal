#include "dvc_dm_imu.h"
#include <cmath>
#include <cstring>

namespace dm_imu {
namespace {
constexpr float pi = 3.14159265358979323846f;
constexpr float gravity = 9.80665f;
float wrap(float v) { return std::remainder(v, 2.0f * pi); }
unsigned le16(const std::uint8_t *p) { return p[0] | (unsigned(p[1]) << 8); }
float decode(const std::uint8_t *p, float limit) { return float(le16(p))*(2*limit/65535.0f)-limit; }
Vector3 decode3(const std::uint8_t *p, float limit) {
    return {decode(p+2,limit), decode(p+4,limit), decode(p+6,limit)};
}
float norm(Vector3 v) { return std::sqrt(v.x*v.x + v.y*v.y + v.z*v.z); }
}
void Sensor::init(std::uint32_t now) { *this = Sensor{}; started_ = now; }
bool Sensor::consume(std::uint16_t id, const std::uint8_t *p, std::size_t size, std::uint32_t now) {
    if (!p || size != 8 || id != master_id || p[0] < 1 || p[0] > 3 ||
        (p[0] != 1 && p[1] != 0) || (id_known_ && id != feedback_id_)) {
        ++rejected_count; return false;
    }
    feedback_id_ = id; id_known_ = true;
    if (p[0] == 1) {
        // V1.2 CAN mapping is +/-235.2 m/s^2, NOT the physical +/-6g range.
        // Byte 1 is temperature, and is allowed to be nonzero.
        accel_ = decode3(p,235.2f); accel_tick_ = now; ++accel_count;
        fuse(now);
    } else if (p[0] == 2) {
        gyro_ = decode3(p,34.88f); gyro_tick_ = now; ++gyro_count;
    } else {
        yaw_ = decode(p+4,180.0f)*pi/180.0f;
        const bool continuous = euler_count && std::uint32_t(now-yaw_tick_) <= timeout_ms;
        yaw_tick_ = now; ++euler_count;
        if (!zero_valid_) {
            const float acceleration = norm(accel_);
            const bool stationary = gyro_count && accel_count &&
                std::uint32_t(now-gyro_tick_) <= 10 && std::uint32_t(now-accel_tick_) <= 10 &&
                norm(gyro_) < 0.05f && acceleration > 0.9f*gravity && acceleration < 1.1f*gravity &&
                std::uint32_t(now-started_) >= 1000;
            if (!stationary || !continuous) { stable_count_ = 0; bias_sum_ = {}; }
            if (stationary) {
                ++stable_count_; bias_sum_.x += gyro_.x; bias_sum_.y += gyro_.y; bias_sum_.z += gyro_.z;
            }
            // Capture one boot origin and gyro bias; disable/re-enable does not re-zero.
            if (stable_count_ >= 20) {
                const float n = float(stable_count_);
                bias_ = {bias_sum_.x/n,bias_sum_.y/n,bias_sum_.z/n};
                zero_ = yaw_; zero_valid_ = true;
            }
        }
    }
    return true;
}
void Sensor::fuse(std::uint32_t now) {
    if (!zero_valid_ || !gyro_count || !euler_count ||
        std::uint32_t(now-gyro_tick_) > 10 || std::uint32_t(now-yaw_tick_) > timeout_ms) return;
    const Vector3 rate{gyro_.x-bias_.x,gyro_.y-bias_.y,gyro_.z-bias_.z};
    const auto elapsed = std::uint32_t(now-fusion_tick_);
    if (!fusion_ready_ || elapsed > timeout_ms) {
        // After a CAN gap, gyro integration cannot reconstruct missing motion.
        // Bridge that interval with the module's measured yaw delta, preserving our boot origin.
        const float seed = fusion_ready_ ? wrap(fused_yaw_ + wrap(yaw_-last_vendor_yaw_)) : yaw_;
        if (!ahrs_.initialize(accel_,rate,seed)) return;
        fusion_ready_ = true;
    } else if (!ahrs_.update(accel_,rate,float(elapsed)*0.001f)) return;
    fused_yaw_ = ahrs_.yaw(); fused_pitch_ = ahrs_.pitch(); fused_roll_ = ahrs_.roll();
    last_vendor_yaw_ = yaw_; fusion_tick_ = now; ++fusion_count;
}
Snapshot Sensor::snapshot(std::uint32_t now) const {
    Snapshot s;
    s.yaw_rad = fused_yaw_; s.yaw_zero_rad = zero_; s.yaw_delta_rad = wrap(fused_yaw_-zero_);
    s.yaw_rate_rad_s = gyro_.z-bias_.z; s.sample_age_ms = std::uint32_t(now-fusion_tick_);
    s.sequence = fusion_count; s.feedback_id = feedback_id_; s.zero_valid = zero_valid_;
    s.valid = zero_valid_ && fusion_ready_ && s.sample_age_ms <= timeout_ms &&
              std::uint32_t(now-gyro_tick_) <= timeout_ms && std::uint32_t(now-accel_tick_) <= timeout_ms &&
              std::uint32_t(now-yaw_tick_) <= timeout_ms;
    s.raw_accel_mps2 = accel_; s.corrected_accel_mps2 = ahrs_.corrected_accel();
    s.raw_gyro_rad_s = gyro_; s.gyro_bias_rad_s = bias_;
    s.vendor_yaw_rad = yaw_; s.pitch_rad = fused_pitch_; s.roll_rad = fused_roll_;
    s.yaw_accel_rad_s2 = ahrs_.yaw_acceleration(); s.accel_age_ms = std::uint32_t(now-accel_tick_);
    return s;
}
std::size_t request(unsigned slot, std::uint16_t &id, std::uint8_t p[8]) {
    std::memset(p, 0, 8);
    constexpr std::uint8_t reads[] = {2,1,3}; // Each stream is sampled every 6 ms.
    id = device_id; p[0] = 0xcc; p[1] = reads[slot%3]; p[3] = 0xdd; return 8;
}
} // namespace dm_imu
