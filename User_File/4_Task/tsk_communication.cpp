#include "tsk_communication.h"
#include <cmath>

namespace {
constexpr uint16_t queue_size = 2048;
struct RxByte { uint32_t ms; uint8_t value; };
RxByte queue[queue_size];
volatile uint16_t head = 0, tail = 0;
volatile bool overflow_pending = false;
volatile uint32_t overflows = 0;
uint8_t frame[9]{}, used = 0;
uint32_t last_byte_ms = 0, rx_frames = 0, checksum_errors = 0;
Struct_Navigation_Command command;
bool received = false;
uint8_t tx[9]{}; // Static storage: DMA owns this until UART becomes READY.

void receive(uint8_t *data, uint16_t size) {
    if (!data || !size) return;
    const uint16_t free = (tail + queue_size - head - 1) % queue_size;
    if (overflow_pending || size > free) {
        ++overflows;
        overflow_pending = true;
        return;
    }
    const auto now = HAL_GetTick();
    auto next = head;
    for (uint16_t i = 0; i < size; ++i) {
        queue[next] = {now, data[i]};
        next = (next + 1) % queue_size;
    }
    __DMB();
    head = next;
}
uint8_t checksum(const uint8_t *data) {
    uint8_t result = 0;
    for (unsigned i = 0; i < 8; ++i) result ^= data[i];
    return result;
}
float decode(const uint8_t *data) {
    const uint16_t raw = uint16_t(data[0]) | (uint16_t(data[1]) << 8);
    const int32_t signed_value = raw >= 0x8000 ? int32_t(raw) - 65536 : raw;
    return signed_value * 0.001f;
}
void discard() {
    --used;
    memmove(frame, frame + 1, used);
}
void parse(const RxByte &byte) {
    if (used && uint32_t(byte.ms - last_byte_ms) > 100) used = 0;
    last_byte_ms = byte.ms;
    frame[used++] = byte.value;
    while (used) {
        if (frame[0] != 0xAA) { discard(); continue; }
        if (used < 2) return;
        if (frame[1] != 0x06) { discard(); continue; }
        if (used < sizeof(frame)) return;
        if (checksum(frame) != frame[8]) { ++checksum_errors; discard(); continue; }
        Struct_Navigation_Command next;
        next.vx_m_s = decode(frame + 2);
        next.vy_m_s = decode(frame + 4);
        next.wz_rad_s = decode(frame + 6);
        next.enable = true;
        next.sequence = static_cast<uint16_t>(++rx_frames);
        next.received_ms = byte.ms; // Arrival time, not foreground processing time.
        const auto mask = __get_PRIMASK();
        __disable_irq();
        command = next;
        received = true;
        __set_PRIMASK(mask);
        used = 0;
    }
}
}
void Communication_Init() {
    head = tail = used = 0;
    overflow_pending = false;
    overflows = rx_frames = checksum_errors = 0;
    command = {};
    received = false;
    UART_Init(&huart1, receive);
}
void Communication_Task() {
    for (unsigned i = 0; i < queue_size; ++i) {
        const auto mask = __get_PRIMASK();
        __disable_irq();
        if (overflow_pending) {
            tail = head;
            used = 0;
            received = false;
            overflow_pending = false;
        }
        if (tail == head) { __set_PRIMASK(mask); break; }
        const auto byte = queue[tail];
        tail = (tail + 1) % queue_size;
        __set_PRIMASK(mask);
        parse(byte);
    }
}
bool Navigation_Get_Command(Struct_Navigation_Command &out) {
    const auto mask = __get_PRIMASK();
    __disable_irq();
    out = command;
    const bool valid = received && !overflow_pending;
    __set_PRIMASK(mask);
    if (!valid || uint32_t(HAL_GetTick() - out.received_ms) >= 200) {
        out = {};
        return false;
    }
    return true;
}
void Navigation_Apply_To_Chassis(chassis_link::Input &input) {
    if (input.left_switch != 0) return;
    Struct_Navigation_Command nav;
    const bool valid = input.remote_online && Navigation_Get_Command(nav);
    // On the robot, the original +vx moved left and +vy moved backward.
    // Compensate by rotating navigation translation 90 degrees clockwise.
    input.vx = valid ? nav.vy_m_s : 0.0f;
    input.vy = valid ? -nav.vx_m_s : 0.0f;
    input.wz = valid ? nav.wz_rad_s : 0.0f;
    // Keep remote status, sequence and interboard re-arm gates intact.
    // Existing chassis encoder limits vx/vy to +/-0.3 and wz to +/-0.8.
}
Struct_Navigation_Statistics Navigation_Get_Statistics() {
    const auto mask = __get_PRIMASK();
    __disable_irq();
    const Struct_Navigation_Statistics result{rx_frames, checksum_errors, overflows};
    __set_PRIMASK(mask);
    return result;
}
HAL_StatusTypeDef Navigation_Send_Velocity(float vx, float vy, float wz) {
    const float values[] = {vx, vy, wz};
    for (auto value : values) if (!std::isfinite(value)) return HAL_ERROR;
    if (huart1.gState != HAL_UART_STATE_READY) return HAL_BUSY;
    tx[0] = 0xAA; tx[1] = 0x06;
    for (unsigned i = 0; i < 3; ++i) {
        const auto bounded = fmaxf(-32.767f, fminf(32.767f, values[i]));
        const auto raw = static_cast<uint16_t>(static_cast<int16_t>(std::lround(bounded * 1000.0f)));
        tx[2 + 2*i] = static_cast<uint8_t>(raw);
        tx[3 + 2*i] = static_cast<uint8_t>(raw >> 8);
    }
    tx[8] = checksum(tx);
    return HAL_UART_Transmit_DMA(&huart1, tx, sizeof(tx));
}
