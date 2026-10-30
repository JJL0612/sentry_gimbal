#include "dvc_chassis_link.h"
#include <cmath>
#include <cstring>

namespace chassis_link {
namespace {
std::uint16_t get16(const std::uint8_t *p) { return p[0] | (std::uint16_t(p[1]) << 8); }
std::uint32_t get32(const std::uint8_t *p) {
    return get16(p) | (std::uint32_t(get16(p + 2)) << 16);
}
std::uint64_t get64(const std::uint8_t *p) {
    return get32(p) | (std::uint64_t(get32(p + 4)) << 32);
}
void put16(std::uint8_t *p, std::uint16_t v) { p[0] = v; p[1] = v >> 8; }
void put32(std::uint8_t *p, std::uint32_t v) { put16(p, v); put16(p + 2, v >> 16); }
void put64(std::uint8_t *p, std::uint64_t v) { put32(p, v); put32(p + 4, v >> 32); }
void putFloat(std::uint8_t *p, float v) {
    static_assert(sizeof(float) == 4, "wire float must be 32 bits");
    std::uint32_t bits; std::memcpy(&bits, &v, 4); put32(p, bits);
}
bool after(std::uint32_t n, std::uint32_t old) { return n != old && n - old < 0x80000000u; }
float limit(float v, float max) { return v > max ? max : (v < -max ? -max : v); }
std::size_t frame(std::uint8_t *out, std::uint16_t id, std::uint32_t seq,
                  const std::uint8_t *payload, std::size_t size) {
    out[0] = 0xa5; out[1] = 0x5a; out[2] = 4; out[3] = 1; // GimbalController
    put16(out + 4, id); put16(out + 6, size); put32(out + 8, seq);
    std::memcpy(out + 12, payload, size);
    put16(out + 12 + size, crc16(out, 12 + size));
    return size + 14;
}
}
std::uint16_t crc16(const std::uint8_t *data, std::size_t size) {
    std::uint16_t crc = 0xffff;
    for (std::size_t i = 0; i < size; ++i) {
        crc ^= std::uint16_t(data[i]) << 8;
        for (unsigned b = 0; b < 8; ++b)
            crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : crc << 1;
    }
    return crc;
}
void Link::init(std::uint64_t boot_id) { *this = Link{}; boot_id_ = boot_id; }
bool Link::online(std::uint32_t now) const {
    return heartbeat_valid_ && std::uint32_t(now - heartbeat_ms_) <= 100;
}
void Link::discardPartial() { used_ = 0; }
void Link::drop(std::size_t n) { used_ -= n; std::memmove(rx_, rx_ + n, used_); }
void Link::acceptHeartbeat(const std::uint8_t *p, std::uint32_t seq, std::uint32_t now) {
    const auto boot = get64(p + 12);
    if (p[0] != 2 || p[1] > 6 || p[2] > 1 || p[3] > 1 || !boot) {
        ++rejected_frames_; return;
    }
    const bool new_boot = !heartbeat_valid_ || boot != peer_boot_id_;
    if (!new_boot && !after(seq, heartbeat_seq_)) { ++rejected_frames_; return; }
    if (new_boot || !online(now)) {
        down_observed_ = enabled_ = was_available_ = false;
    }
    peer_boot_id_ = boot;
    peer_reasons_ = get32(p + 4);
    // v4 ready describes actuator status, not permission to send a new target.
    peer_allowed_ = p[1] != 5 && p[1] != 6 && !(peer_reasons_ & (1u << 6));
    heartbeat_seq_ = seq; heartbeat_ms_ = now; heartbeat_valid_ = true;
    ++valid_frames_;
}
void Link::scan(std::uint32_t now) {
    while (used_) {
        if (rx_[0] != 0xa5 || (used_ >= 2 && rx_[1] != 0x5a)) { drop(1); continue; }
        if (used_ < 12) return;
        const auto n = get16(rx_ + 6);
        if (rx_[2] != 4 || rx_[3] != 2 || n > 128) {
            ++rejected_frames_; drop(1); continue;
        }
        const std::size_t total = n + 14;
        if (used_ < total) return;
        if (get16(rx_ + total - 2) != crc16(rx_, total - 2)) {
            ++rejected_frames_; drop(1); continue;
        }
        const auto id = get16(rx_ + 4);
        if (id == 1 && n == 20)
            acceptHeartbeat(rx_ + 12, get32(rx_ + 8), now);
        else if (!((id == 0x0103 && n == 32) || (id == 0x0403 && n == 24)))
            ++rejected_frames_;
        // Feedback may share the stream; it never refreshes the heartbeat watchdog.
        drop(total);
        assembly_ms_ = now;
    }
}
void Link::consume(const std::uint8_t *p, std::size_t size, std::uint32_t now) {
    if (used_ && std::uint32_t(now - assembly_ms_) > 30) discardPartial();
    if (!p) return;
    for (std::size_t i = 0; i < size; ++i) {
        if (!used_) assembly_ms_ = now;
        if (used_ == sizeof(rx_)) drop(1);
        rx_[used_++] = p[i]; scan(now);
    }
}
void Link::update(const Input &in, std::uint32_t now) {
    const bool previous_enabled = enabled_;
    const bool previous_yaw_permission = input_.big_yaw_permission_enabled;
    input_ = in; input_ms_ = now;
    const bool finite = std::isfinite(in.vx) && std::isfinite(in.vy) && std::isfinite(in.wz) && (!in.big_yaw_present || std::isfinite(in.big_yaw_rate_rad_s));
    const bool available = boot_id_ && online(now) && peer_allowed_ && in.remote_online && finite;
    if (!available || !was_available_) down_observed_ = false;
    if (available && in.left_switch == -1) down_observed_ = true;
    enabled_ = available && down_observed_ && (in.left_switch == 0 || in.left_switch == 1);
    was_available_ = available;
    if (in.remote_sequence != last_remote_sequence_ || previous_enabled != enabled_ ||
        previous_yaw_permission != in.big_yaw_permission_enabled) ++command_sequence_;
    last_remote_sequence_ = in.remote_sequence;
}
std::size_t Link::encode(std::uint8_t *out, std::size_t cap, std::uint32_t now) {
    const auto required = peer_boot_id_ ? (command_batch_size + (input_.imu_present ? imu_frame_size : 0) +
                                          (input_.big_yaw_present ? big_yaw_frame_size : 0)) : heartbeat_frame_size;
    if (!out || cap < required || !boot_id_) return 0;
    // Re-evaluate freshness even if a caller skipped update().
    if (!online(now)) {
        if (enabled_) ++command_sequence_;
        enabled_ = down_observed_ = was_available_ = false;
    }
    const bool active = enabled_;
    std::uint8_t h[20]{};
    h[0] = 1; h[1] = active ? 3 : 2; h[2] = input_.remote_online;
    h[3] = !online(now); put32(h + 4, active ? 0 : 1); put32(h + 8, now);
    put64(h + 12, boot_id_);
    std::size_t n = frame(out, 1, ++tx_sequence_, h, sizeof(h));
    // v4 requires a nonzero destination identity even for Disabled controls.
    if (!peer_boot_id_) return n;
    std::uint8_t c[32]{};
    c[0] = active ? 1 : 0; c[1] = 1; c[2] = active ? 2 : 0;
    putFloat(c + 4, active ? limit(input_.vx, 0.3f) : 0);
    putFloat(c + 8, active ? limit(input_.vy, 0.3f) : 0);
    // Remote spin supports 2.0..3.4 rad/s; preserve the navigation limit.
    const float yaw_limit = input_.left_switch == 1 ? 3.4f : 0.8f;
    putFloat(c + 12, active ? limit(input_.wz, yaw_limit) : 0);
    put32(c + 16, active ? 0 : 1);
    // Advance on a real remote sample or a safety transition; retransmits retain the sequence.
    put32(c + 20, command_sequence_);
    put64(c + 24, peer_boot_id_);
    n += frame(out + n, 0x0101, ++tx_sequence_, c, sizeof(c));
    if (input_.imu_present) {
        std::uint8_t imu[32]{};
        const auto elapsed = std::uint32_t(now - input_ms_);
        const bool valid = input_.imu_valid && elapsed <= 50 &&
                           input_.imu_sample_age_ms <= 50 - elapsed &&
                           std::isfinite(input_.imu_yaw_delta_rad) && std::isfinite(input_.imu_yaw_rate_rad_s);
        imu[0] = valid;
        putFloat(imu + 4, valid ? input_.imu_yaw_delta_rad : 0);
        putFloat(imu + 8, valid ? input_.imu_yaw_rate_rad_s : 0);
        put32(imu + 12, valid ? input_.imu_sample_age_ms + elapsed : 0);
        put32(imu + 16, input_.imu_sequence);
        put32(imu + 20, input_.imu_zero_generation); // Onboard EKF restart changes this generation; board reboot changes heartbeat boot ID
        put64(imu + 24, peer_boot_id_);
        n += frame(out + n, 0x0404, ++tx_sequence_, imu, sizeof(imu));
    }
    if (input_.big_yaw_present) {
        std::uint8_t yaw[36]{};
        const auto age = std::uint32_t(now - input_ms_);
        const bool permitted = active && input_.big_yaw_permission_enabled &&
                               age <= 100 && std::isfinite(input_.big_yaw_rate_rad_s);
        yaw[0] = permitted ? 1 : 0; yaw[1] = 1; yaw[2] = permitted;
        putFloat(yaw + 4, permitted ? limit(input_.big_yaw_rate_rad_s, 1.0f) : 0);
        put32(yaw + 8, command_sequence_);
        put32(yaw + 12, age); put32(yaw + 16, age);
        put64(yaw + 20, peer_boot_id_); put32(yaw + 28, age);
        put32(yaw + 32, command_sequence_);
        n += frame(out + n, 0x0402, ++tx_sequence_, yaw, sizeof(yaw));
    }
    return n;
}
} // namespace chassis_link
