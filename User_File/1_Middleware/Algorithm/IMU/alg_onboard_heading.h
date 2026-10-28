#pragma once
#include <algorithm>
#include <cmath>
#include <cstdint>
#include "alg_gimbal_ahrs.h"

namespace gimbal_imu {
// Board is behind and to the right of big yaw; sensor axes X forward/Y left/Z up.
constexpr float board_arm_x_m = -0.04360f, board_arm_y_m = -0.04996f;
class PlanarLeverArm {
public:
    void observe_rate(float rate, std::uint64_t sample_us) {
        if (!std::isfinite(rate) || sample_us == previous_us_) return;
        if (previous_us_ && sample_us > previous_us_) {
            const float dt = float(sample_us-previous_us_)*1e-6f;
            if (dt <= 0.05f) {
                const float derivative = std::clamp((rate-previous_rate_)/dt,-200.0f,200.0f);
                alpha_ += dt/(0.020f+dt)*(derivative-alpha_);
            } else alpha_=0;
        } else alpha_=0;
        previous_us_=sample_us; previous_rate_=rate;
    }
    dm_imu::Vector3 correct(dm_imu::Vector3 raw) const {
        const float w2=previous_rate_*previous_rate_;
        return {raw.x+alpha_*board_arm_y_m+w2*board_arm_x_m,
                raw.y-alpha_*board_arm_x_m+w2*board_arm_y_m,raw.z};
    }
    float alpha() const { return alpha_; }
private:
    std::uint64_t previous_us_=0;
    float previous_rate_=0,alpha_=0;
};

struct HeadingSample {
    float yaw_rad=0, yaw_rate_rad_s=0, accel_norm_mps2=0;
    std::uint64_t sample_us=0;
    std::uint32_t sequence=0,generation=0;
    bool healthy=false;
};
struct HeadingOutput {
    float delta_rad=0,rate_rad_s=0;
    std::uint32_t age_ms=0,sequence=0,generation=0;
    bool valid=false;
};
class HeadingOrigin {
public:
    void init(std::uint32_t now_ms) { *this=HeadingOrigin{};started_ms_=now_ms; }
    HeadingOutput update(HeadingSample s,std::uint64_t now_us,std::uint32_t now_ms) {
        HeadingOutput out;
        out.sequence=s.sequence;out.generation=s.generation;out.rate_rad_s=s.yaw_rate_rad_s;
        const bool finite=std::isfinite(s.yaw_rad)&&std::isfinite(s.yaw_rate_rad_s)&&
                          std::isfinite(s.accel_norm_mps2);
        const bool ordered=now_us>=s.sample_us;
        const auto age=ordered?(now_us-s.sample_us)/1000:UINT64_MAX;
        out.age_ms=age<=UINT32_MAX?std::uint32_t(age):UINT32_MAX;
        const bool fresh=s.healthy&&finite&&s.sample_us&&s.sequence&&s.generation&&age<=50;
        if (s.generation!=generation_) {
            generation_=s.generation;zero_valid_=false;stable_=0;previous_sequence_=0;
        }
        if (!fresh) { stable_=0;return out; }
        if (s.sequence!=previous_sequence_) {
            const bool continuous=previous_sample_us_&&s.sample_us>previous_sample_us_&&
                                  s.sample_us-previous_sample_us_<=50000;
            const bool stationary=std::fabs(s.yaw_rate_rad_s)<0.05f&&
                s.accel_norm_mps2>0.9f*9.80665f&&s.accel_norm_mps2<1.1f*9.80665f&&
                std::uint32_t(now_ms-started_ms_)>=1000;
            if (!zero_valid_) {
                stable_=stationary?(continuous?stable_+1:1):0;
                if(stable_>=20) { zero_=s.yaw_rad;zero_valid_=true; }
            }
            previous_sequence_=s.sequence;previous_sample_us_=s.sample_us;
        }
        out.delta_rad=std::remainder(s.yaw_rad-zero_,6.2831853071795864769f);
        out.valid=zero_valid_;return out;
    }
private:
    float zero_=0;
    std::uint32_t started_ms_=0,generation_=0,previous_sequence_=0,stable_=0;
    std::uint64_t previous_sample_us_=0;
    bool zero_valid_=false;
};
} // namespace gimbal_imu
