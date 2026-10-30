#pragma once
#include <algorithm>
#include <cmath>
#include <cstdint>

namespace yaw_follow {
struct SmallImu {
    bool valid = false;
    float heading_rad = 0, rate_rad_s = 0;
    std::uint32_t age_ms = 0, gyro_age_ms = 0, sequence = 0;
};
struct Output {
    bool valid = false;
    float small_target_rad = 0, big_world_rate_rad_s = 0;
    float relative_rate_feedforward = 0, measured_relative_rate = 0;
    float world_error_rad = 0;
};
// Big yaw uses onboard BMI088; small yaw uses its own external world heading.
// The encoder is used only for joint travel and big-yaw recentering.
class Controller {
public:
    static constexpr float center_rad = 0.765f;
    static constexpr float min_rad = 0.0f, max_rad = 1.53f;
    // Physical CCW sign of increasing small-yaw encoder angle; verify on robot.
    static constexpr float small_direction = 1.0f;
    static constexpr float small_imu_direction = 1.0f; // External sensor +Z is CCW.
    static constexpr float stick_max_rate = 1.0f;
    static constexpr float follow_kp = 2.0f, follow_max_rate = 1.0f;
    static constexpr float follow_deadband = 0.01f;

    Output update(bool enabled, float big_heading, float big_rate,
                  std::uint32_t imu_age_ms, std::uint32_t generation,
                  bool encoder_fresh, float small_angle, float stick,
                  std::uint32_t now, const SmallImu &imu) {
        if (!enabled || !encoder_fresh || imu_age_ms > 50 || !imu.valid ||
            imu.age_ms > 50 || imu.gyro_age_ms > 50 || !imu.sequence ||
            !std::isfinite(imu.heading_rad) || !std::isfinite(imu.rate_rad_s) ||
            !std::isfinite(big_heading) || !std::isfinite(big_rate) ||
            !std::isfinite(small_angle) || !std::isfinite(stick)) {
            reset(); return {};
        }
        small_angle = wrap(small_angle);
        if (small_angle < min_rad - 0.005f || small_angle > max_rad + 0.005f) {
            reset(); return {};
        }
        // The two IMUs have independent yaw zeros: never subtract absolute headings.
        // Predict the external heading only from its measured sample age.
        const float heading = wrap(small_imu_direction * (imu.heading_rad +
            imu.rate_rad_s * float(imu.age_ms) * 0.001f));
        const float world_rate = small_imu_direction * imu.rate_rad_s;
        const auto elapsed = std::uint32_t(now - last_ms_);
        const bool capture = !initialized_ || generation_ != generation || elapsed > 50 ||
            (last_imu_sequence_ && static_cast<std::int32_t>(imu.sequence-last_imu_sequence_) < 0);
        const float dt = capture ? 0.0f : float(elapsed) * 0.001f;
        if (capture) desired_heading_ = heading;
        initialized_ = true; generation_ = generation; last_ms_ = now;
        last_imu_sequence_ = imu.sequence;
        stick = std::clamp(stick, -1.0f, 1.0f);
        if (std::fabs(stick) < 0.05f) stick = 0;
        desired_heading_ = wrap(desired_heading_ + small_direction * stick * stick_max_rate * dt);
        // Project actual small-yaw WORLD error into a reachable joint target.
        // A moving big yaw cannot hide gun drift behind a correct encoder angle.
        const float world_error = wrap(desired_heading_ - heading);
        const float target = std::clamp(small_angle + small_direction * world_error,
                                        min_rad, max_rad);
        desired_heading_ = wrap(heading + small_direction * (target - small_angle));
        // Feedforward is JOINT rate; feedback uses the same coordinate. Their
        // difference is desired-minus-measured small-yaw world rate.
        float desired_world_rate = small_direction * stick * stick_max_rate;
        if ((target >= max_rad && small_direction * desired_world_rate > 0) ||
            (target <= min_rad && small_direction * desired_world_rate < 0)) desired_world_rate = 0;
        const float error = small_direction * (small_angle - center_rad);
        const float rate = std::fabs(error) <= follow_deadband ? 0.0f :
            std::clamp(follow_kp * error, -follow_max_rate, follow_max_rate);
        return {true, target, rate,
            small_direction * (desired_world_rate - big_rate),
            small_direction * (world_rate - big_rate),
            wrap(desired_heading_ - heading)};
    }
    void reset() { initialized_ = false; }
private:
    static float wrap(float angle) {
        constexpr float pi = 3.14159265358979323846f;
        return std::remainder(angle, 2.0f * pi);
    }
    bool initialized_ = false;
    std::uint32_t generation_ = 0, last_ms_ = 0, last_imu_sequence_ = 0;
    float desired_heading_ = 0;
};
} // namespace yaw_follow
