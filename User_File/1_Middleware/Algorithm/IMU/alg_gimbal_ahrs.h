#pragma once

namespace dm_imu {
// Body axes: X forward, Y left, Z up. Planar yaw-axis compensation only.
// The measured horizontal arm is from the big-yaw axis to the IMU.
constexpr float lever_forward_m = 0.069f;
constexpr float lever_left_m = -0.106f;
struct Vector3 { float x = 0, y = 0, z = 0; };

class GimbalAhrs {
public:
    bool initialize(Vector3 accel, Vector3 gyro, float yaw_rad);
    bool update(Vector3 accel, Vector3 gyro, float dt_s);
    float yaw() const;
    float pitch() const;
    float roll() const;
    Vector3 corrected_accel() const { return corrected_; }
    float yaw_acceleration() const { return alpha_; }
private:
    void compensate(Vector3 accel, float yaw_rate);
    float q_[4] = {1, 0, 0, 0}; // Sensor/body -> world, w,x,y,z.
    float previous_rate_ = 0, alpha_ = 0;
    Vector3 corrected_{};
};
} // namespace dm_imu
