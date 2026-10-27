#include "2_Device/Communication/dvc_chassis_link.h"
extern "C" {
#include "alg_board_link.h"
#include "app_gimbal_link.h"
}
#include <cassert>
#include <cmath>
#include <iostream>
#include <limits>
#include <vector>

static std::uint32_t clock_ms;
extern "C" std::uint32_t HAL_GetTick(void) { return clock_ms; }
namespace {
std::vector<std::uint8_t> heartbeat(std::uint32_t seq, std::uint64_t boot = 42,
                                   bool ready = false, unsigned state = kBoardLinkSafetyWaiting) {
    BoardLinkHeartbeat h{};
    h.role = kBoardLinkRoleChassisController; h.sender_boot_id = boot;
    h.ready = ready; h.safety_state = state;
    std::vector<std::uint8_t> bytes(chassis_link::heartbeat_frame_size);
    std::uint16_t size = 0;
    assert(BoardLinkEncodeHeartbeat(&h, seq, bytes.data(), bytes.size(), &size));
    assert(size == bytes.size());
    return bytes;
}
void receive(chassis_link::Link &l, const std::vector<std::uint8_t> &b, std::uint32_t now) {
    l.consume(b.data(), b.size(), now);
}
void repairCrc(std::vector<std::uint8_t> &b) {
    auto c = BoardLinkCrc16(b.data(), b.size() - 2);
    b[b.size()-2] = c; b.back() = c >> 8;
}
BoardLinkChassisControl decode(chassis_link::Link &l, std::uint32_t now) {
    std::uint8_t bytes[chassis_link::batch_size];
    assert(l.encode(bytes, sizeof(bytes), now) == sizeof(bytes));
    BoardLinkParser parser{}; BoardLinkParserInit(&parser);
    for (auto b : bytes) BoardLinkParserFeed(&parser, &b, 1, now);
    BoardLinkFrame f{}; BoardLinkHeartbeat h{};
    assert(BoardLinkParserPop(&parser, &f) && BoardLinkDecodeHeartbeat(&f, &h));
    assert(h.sender_boot_id == 123 && h.role == kBoardLinkRoleGimbalController);
    BoardLinkChassisControl c{};
    assert(BoardLinkParserPop(&parser, &f) && BoardLinkDecodeChassisControl(&f, &c));
    assert(!BoardLinkParserPop(&parser, &f));
    // Exercise the actual chassis application RX consumer, not only its codec.
    clock_ms = now;
    for (auto b : bytes) GimbalLinkRxCallback(&b, 1);
    BoardLinkChassisControl actual{};
    assert(GimbalLinkPeerOnline() && GimbalLinkGetChassisControl(&actual));
    assert(actual.command_sequence == c.command_sequence);
    assert(actual.receiver_boot_id == c.receiver_boot_id);
    assert(actual.global_action == c.global_action);
    assert(actual.vx_m_s == c.vx_m_s && actual.vy_m_s == c.vy_m_s && actual.wz_rad_s == c.wz_rad_s);
    return c;
}
}
int main() {
    GimbalLinkInit();
    const std::uint8_t vector[] = {'1','2','3','4','5','6','7','8','9'};
    assert(chassis_link::crc16(vector, 9) == 0x29b1);
    chassis_link::Link l; l.init(123);
    chassis_link::Input in{};
    in.remote_online = true; in.left_switch = 1; in.remote_sequence = 10;
    in.vx = 0.2f; in.vy = -0.1f; in.wz = 0.4f;
    l.update(in, 0); assert(!l.enabled());
    // Bootstrap must not emit an invalid receiver_boot_id=0 control.
    std::uint8_t initial[chassis_link::batch_size];
    assert(l.encode(initial, sizeof(initial), 0) == 34);
    assert(initial[2] == 4 && initial[6] == 20);
    assert(l.encode(initial, 33, 0) == 0);
    auto h = heartbeat(1); // Actual chassis reports ready=false while waiting for targets.
    for (auto b : h) l.consume(&b, 1, 1);
    assert(l.online(1));
    l.update(in, 1); assert(!l.enabled());
    assert(decode(l, 1).global_action == 0);
    in.left_switch = -1; ++in.remote_sequence; l.update(in, 2);
    in.left_switch = 0; ++in.remote_sequence; l.update(in, 3);
    assert(l.enabled()); // ready=false must not deadlock command production.
    auto c = decode(l, 3);
    assert(c.mode == kBoardLinkModeBodyVelocity && c.global_action == 2 && c.source == 1);
    assert(c.receiver_boot_id == 42);
    assert(std::fabs(c.vx_m_s - 0.2f) < 1e-6f && std::fabs(c.vy_m_s + 0.1f) < 1e-6f);
    assert(std::fabs(c.wz_rad_s - 0.4f) < 1e-6f);
    assert(decode(l, 4).command_sequence == c.command_sequence);
    ++in.remote_sequence; l.update(in, 5);
    assert(decode(l, 5).command_sequence > c.command_sequence);
    c = decode(l, 6);
    in.remote_online = false; l.update(in, 7);
    auto disabled = decode(l, 7);
    assert(disabled.global_action == 0 && disabled.vx_m_s == 0 && disabled.vy_m_s == 0 && disabled.wz_rad_s == 0);
    assert(disabled.command_sequence > c.command_sequence);
    in.remote_online = true; l.update(in, 8); assert(!l.enabled());
    in.left_switch = -1; l.update(in, 9); in.left_switch = 1; l.update(in, 10);
    assert(l.enabled());
    // Duplicates/out-of-order heartbeats cannot keep a stale session online.
    receive(l, heartbeat(1), 80); receive(l, heartbeat(0), 90);
    auto seq = decode(l, 100).command_sequence;
    disabled = decode(l, 102); // encode alone also performs timeout revocation.
    assert(!l.online(102) && !l.enabled() && disabled.command_sequence > seq);
    receive(l, heartbeat(2), 103); l.update(in, 103); assert(!l.enabled());
    in.left_switch = -1; l.update(in, 104); in.left_switch = 1; l.update(in, 105);
    assert(l.enabled());
    // ready changes are observational, unlike explicit emergency/config-blocked states.
    receive(l, heartbeat(3, 42, true), 106); l.update(in, 106); assert(l.enabled());
    receive(l, heartbeat(4, 42, false), 107); l.update(in, 107); assert(l.enabled());
    receive(l, heartbeat(5, 42, false, kBoardLinkSafetyEmergencyStop), 108);
    l.update(in, 108); assert(!l.enabled());
    receive(l, heartbeat(6, 42), 109); l.update(in, 109); assert(!l.enabled());
    in.left_switch = -1; l.update(in, 110); in.left_switch = 1; l.update(in, 111);
    receive(l, heartbeat(7, 42, false, kBoardLinkSafetyConfigBlocked), 112);
    l.update(in, 112); assert(!l.enabled());
    receive(l, heartbeat(1, 99), 113); l.update(in, 113);
    assert(!l.enabled() && decode(l, 113).receiver_boot_id == 99);
    auto valid = l.valid_frames();
    auto corrupt = heartbeat(2, 99); corrupt[16] ^= 1; receive(l, corrupt, 114);
    assert(l.valid_frames() == valid);
    // Reject v1, wrong direction, malformed heartbeat lengths, and invalid fields.
    for (unsigned kind = 0; kind < 5; ++kind) {
        auto bad = heartbeat(2, 99);
        if (kind == 0) bad[2] = 1;
        if (kind == 1) bad[3] = 1;
        if (kind == 2) { bad.insert(bad.end()-2, 4, 0); bad[6] = 24; }
        if (kind == 3) bad[14] = 2; // invalid ready flag
        if (kind == 4) bad[12] = 1; // payload role differs from header
        repairCrc(bad); receive(l, bad, 115 + kind);
        assert(l.valid_frames() == valid);
    }
    std::uint8_t noise[] = {0, 0xa5, 0, 0xff}; l.consume(noise, sizeof(noise), 120);
    receive(l, heartbeat(2, 99), 121); assert(l.valid_frames() == valid + 1);
    auto split = heartbeat(3, 99);
    l.consume(split.data(), 8, 122); l.consume(split.data()+8, split.size()-8, 153);
    assert(l.valid_frames() == valid + 1);
    receive(l, split, 154); assert(l.valid_frames() == valid + 2);
    // A valid feedback frame shares the stream but cannot refresh heartbeat age.
    BoardLinkChassisFeedback fb{}; fb.execution_state = kBoardLinkExecutionWaiting;
    std::uint8_t fb_bytes[kBoardLinkMaxFrameSize]; std::uint16_t fb_size = 0;
    assert(BoardLinkEncodeChassisFeedback(&fb, 4, fb_bytes, sizeof(fb_bytes), &fb_size));
    l.consume(fb_bytes, fb_size, 255); assert(!l.online(255));
    receive(l, heartbeat(4, 99), 256);
    in.left_switch = -1; l.update(in, 257); in.left_switch = 1;
    in.vx = 100; in.vy = -100; in.wz = 100; l.update(in, 258);
    c = decode(l, 258);
    assert(c.vx_m_s == 0.3f && c.vy_m_s == -0.3f && c.wz_rad_s == 0.8f);
    in.vx = std::numeric_limits<float>::quiet_NaN(); l.update(in, 259);
    assert(!l.enabled());
    assert(decode(l, 259).vx_m_s == 0);
    chassis_link::Link no_entropy; no_entropy.init(0);
    assert(no_entropy.encode(initial, sizeof(initial), 1) == 0);
    chassis_link::Link wrap; wrap.init(123);
    receive(wrap, heartbeat(0xffffffffu), 0xfffffff0u);
    receive(wrap, heartbeat(0), 4);
    assert(wrap.online(4) && wrap.valid_frames() == 2);
    // Captured hardware heartbeat: version 4, 20 bytes, CRC 0x8321.
    const std::uint8_t captured[] = {0xa5,0x5a,4,2,1,0,20,0,0xa2,0xc6,0,0,
        2,1,0,1,0,0,0,0,0xfd,0xb4,3,0,0x4a,0x28,0x4c,0x65,0x57,0x59,0x4b,0x53,0x21,0x83};
    chassis_link::Link hw; hw.init(123); hw.consume(captured, sizeof(captured), 1);
    assert(hw.online(1) && hw.valid_frames() == 1);
    std::cout << "v4 compatibility, chassis consumer and recovery tests passed\n";
}
