#pragma once
#include <cstddef>
#include <cstdint>

// SkyWalker interboard v4. Pure protocol/state machine; no HAL dependencies.
namespace chassis_link {
inline constexpr std::size_t heartbeat_frame_size = 34, control_frame_size = 46;
inline constexpr std::size_t batch_size = heartbeat_frame_size + control_frame_size;
struct Input {
    bool remote_online = false;
    // -1 down (disable), 0 middle (chassis only), 1 up (chassis + local joints).
    int left_switch = -1;
    std::uint32_t remote_sequence = 0;
    float vx = 0, vy = 0, wz = 0;
};
class Link {
public:
    void init(std::uint64_t boot_id);
    void consume(const std::uint8_t *bytes, std::size_t size, std::uint32_t now);
    void discardPartial();
    // Called every 1ms. After boot/loss/recovery, observe DOWN before enabling.
    void update(const Input &input, std::uint32_t now);
    // 34-byte heartbeat + 46-byte control; heartbeat only until peer boot is known.
    // Call every 20ms when UART TX is idle.
    std::size_t encode(std::uint8_t *out, std::size_t capacity, std::uint32_t now);
    bool online(std::uint32_t now) const;
    bool enabled() const { return enabled_; }
    std::uint32_t peer_reasons() const { return peer_reasons_; }
    std::uint32_t valid_frames() const { return valid_frames_; }
    std::uint32_t rejected_frames() const { return rejected_frames_; }
private:
    void scan(std::uint32_t now);
    void acceptHeartbeat(const std::uint8_t *p, std::uint32_t sequence, std::uint32_t now);
    void drop(std::size_t size);
    std::uint8_t rx_[142]{};
    std::size_t used_ = 0;
    std::uint32_t assembly_ms_ = 0, heartbeat_ms_ = 0, heartbeat_seq_ = 0;
    std::uint32_t peer_reasons_ = 0;
    std::uint32_t tx_sequence_ = 0, last_remote_sequence_ = 0, command_sequence_ = 1;
    std::uint32_t valid_frames_ = 0, rejected_frames_ = 0;
    std::uint64_t boot_id_ = 0, peer_boot_id_ = 0;
    bool heartbeat_valid_ = false, peer_allowed_ = false;
    bool down_observed_ = false, enabled_ = false, was_available_ = false;
    Input input_{};
};
std::uint16_t crc16(const std::uint8_t *data, std::size_t size);
} // namespace chassis_link
