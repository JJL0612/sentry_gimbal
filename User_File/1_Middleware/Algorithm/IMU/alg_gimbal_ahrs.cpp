#include "alg_gimbal_ahrs.h"
#include <algorithm>
#include <cmath>

namespace dm_imu {
namespace {
constexpr float gravity = 9.80665f;
constexpr float mahony_kp = 0.2f; // Ki = 0: gravity cannot observe yaw bias.
constexpr float alpha_time_constant_s = 0.020f;
bool finite(Vector3 v) { return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z); }
float norm(Vector3 v) { return std::sqrt(v.x*v.x + v.y*v.y + v.z*v.z); }
}
void GimbalAhrs::compensate(Vector3 accel, float yaw_rate) {
    // f_axis = f_imu - alpha x r - omega x (omega x r).
    // This preserves gravity (specific force); it is NOT world linear acceleration.
    // No pitch joint term: the user confirmed that this IMU does not follow pitch.
    const float w2 = yaw_rate * yaw_rate;
    corrected_ = {accel.x + alpha_*lever_left_m + w2*lever_forward_m,
                  accel.y - alpha_*lever_forward_m + w2*lever_left_m, accel.z};
}
bool GimbalAhrs::initialize(Vector3 accel, Vector3 gyro, float yaw_rad) {
    if (!finite(accel) || !finite(gyro) || !std::isfinite(yaw_rad)) return false;
    alpha_ = 0; previous_rate_ = gyro.z;
    compensate(accel, gyro.z);
    const float n = norm(corrected_);
    if (n < 0.8f*gravity || n > 1.2f*gravity) return false;
    const float roll_rad = std::atan2(corrected_.y, corrected_.z);
    const float pitch_rad = std::atan2(-corrected_.x,
                                      std::hypot(corrected_.y, corrected_.z));
    const float cr = std::cos(roll_rad/2), sr = std::sin(roll_rad/2);
    const float cp = std::cos(pitch_rad/2), sp = std::sin(pitch_rad/2);
    const float cy = std::cos(yaw_rad/2), sy = std::sin(yaw_rad/2);
    q_[0] = cr*cp*cy + sr*sp*sy; q_[1] = sr*cp*cy - cr*sp*sy;
    q_[2] = cr*sp*cy + sr*cp*sy; q_[3] = cr*cp*sy - sr*sp*cy;
    return true;
}
bool GimbalAhrs::update(Vector3 accel, Vector3 gyro, float dt_s) {
    if (!finite(accel) || !finite(gyro) || !std::isfinite(dt_s) ||
        dt_s < 0.001f || dt_s > 0.050f) return false;
    // Differentiate actual measured yaw rate, not wheel/chassis commands.
    const float derivative = std::clamp((gyro.z - previous_rate_)/dt_s, -200.0f, 200.0f);
    alpha_ += dt_s/(alpha_time_constant_s + dt_s)*(derivative - alpha_);
    previous_rate_ = gyro.z;
    compensate(accel, gyro.z);
    const float w=q_[0], x=q_[1], y=q_[2], z=q_[3];
    const float n = norm(corrected_);
    // Suppress gravity correction during large remaining translational acceleration.
    if (n >= 0.8f*gravity && n <= 1.2f*gravity) {
        const Vector3 a{corrected_.x/n, corrected_.y/n, corrected_.z/n};
        const Vector3 g{2*(x*z-w*y), 2*(w*x+y*z), 1-2*(x*x+y*y)};
        gyro.x += mahony_kp*(a.y*g.z-a.z*g.y);
        gyro.y += mahony_kp*(a.z*g.x-a.x*g.z);
        gyro.z += mahony_kp*(a.x*g.y-a.y*g.x);
    }
    const float half_dt = 0.5f*dt_s;
    q_[0] += (-x*gyro.x-y*gyro.y-z*gyro.z)*half_dt;
    q_[1] += ( w*gyro.x+y*gyro.z-z*gyro.y)*half_dt;
    q_[2] += ( w*gyro.y-x*gyro.z+z*gyro.x)*half_dt;
    q_[3] += ( w*gyro.z+x*gyro.y-y*gyro.x)*half_dt;
    const float qn = std::sqrt(q_[0]*q_[0]+q_[1]*q_[1]+q_[2]*q_[2]+q_[3]*q_[3]);
    if (!std::isfinite(qn) || qn < 0.5f) return false;
    for (auto &component : q_) component /= qn;
    return true;
}
float GimbalAhrs::yaw() const {
    return std::atan2(2*(q_[0]*q_[3]+q_[1]*q_[2]), 1-2*(q_[2]*q_[2]+q_[3]*q_[3]));
}
float GimbalAhrs::pitch() const {
    return std::asin(std::clamp(2*(q_[0]*q_[2]-q_[3]*q_[1]), -1.0f, 1.0f));
}
float GimbalAhrs::roll() const {
    return std::atan2(2*(q_[0]*q_[1]+q_[2]*q_[3]), 1-2*(q_[1]*q_[1]+q_[2]*q_[2]));
}
} // namespace dm_imu
