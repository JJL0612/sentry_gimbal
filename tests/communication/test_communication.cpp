#include "4_Task/tsk_communication.h"
#include <cassert>
#include <cstdio>
#include <limits>
#include <vector>

UART_HandleTypeDef huart1, huart7, huart10;
static uint32_t tick;
static UART_Callback board_rx, nav_rx;
static uint8_t *active_tx;
static std::vector<uint8_t> sent;
uint32_t HAL_GetTick() { return tick; }
void UART_Init(UART_HandleTypeDef *uart, UART_Callback callback)
{
    assert(uart == &huart1 || uart == &huart7); // USART10 belongs to the chassis link.
    (uart == &huart1 ? board_rx : nav_rx) = callback;
}
HAL_StatusTypeDef HAL_UART_Transmit_DMA(UART_HandleTypeDef *uart, uint8_t *data, uint16_t length)
{
    if (uart->gState != HAL_UART_STATE_READY) return HAL_BUSY;
    sent.assign(data, data + length);
    active_tx = data;
    uart->gState = HAL_UART_STATE_BUSY_TX;
    return HAL_OK;
}

static uint16_t crc(const std::vector<uint8_t> &data)
{
    uint16_t value = 0xffff;
    for (uint8_t byte : data)
    {
        value ^= uint16_t(byte) << 8;
        for (int i = 0; i < 8; ++i)
            value = (value & 0x8000) ? uint16_t((value << 1) ^ 0x1021) : uint16_t(value << 1);
    }
    return value;
}
static std::vector<uint8_t> frame(uint8_t type, const std::vector<uint8_t> &payload = {})
{
    std::vector<uint8_t> data = {0xa5, 0x5a, 1, type, uint8_t(payload.size()), 0, 7, 0};
    data.insert(data.end(), payload.begin(), payload.end());
    const uint16_t checksum = crc(data);
    data.push_back(uint8_t(checksum));
    data.push_back(uint8_t(checksum >> 8));
    return data;
}
static uint32_t callbacks;
static void received(uint8_t, uint16_t seq, const uint8_t *, uint16_t)
{
    assert(seq == 7);
    ++callbacks;
}

int main()
{
    // 标准校验向量，与 Python binascii.crc_hqx 一致。
    assert(crc({'1','2','3','4','5','6','7','8','9'}) == 0x29b1);
    UART_HandleTypeDef uart;
    Class_UART_Link link;
    link.Init(&uart, received);
    assert(!link.Is_Online());
    auto data = frame(UART_LINK_BOARD_DATA, {1, 2, 3});
    for (auto byte : data) { link.Receive_From_ISR(&byte, 1); link.Process(); }
    assert(callbacks == 1 && link.Is_Online());
    auto bad = data;
    bad.back() ^= 1;
    bad.insert(bad.end(), data.begin(), data.end());
    bad.insert(bad.end(), data.begin(), data.end());
    link.Receive_From_ISR(bad.data(), bad.size()); link.Process();
    assert(callbacks == 3 && link.Get_CRC_Error_Count() == 1);
    auto invalid_length = frame(0x10);
    invalid_length[4] = 129;
    invalid_length.insert(invalid_length.end(), data.begin(), data.end());
    link.Receive_From_ISR(invalid_length.data(), invalid_length.size()); link.Process();
    assert(callbacks == 4);
    auto max_frame = frame(0x10, std::vector<uint8_t>(128, 0xa5));
    link.Receive_From_ISR(max_frame.data(), max_frame.size()); link.Process();
    assert(callbacks == 5);
    // 连续四帧跨 512 字节 DMA 块边界；验证既不重复，也不遗漏尾部半帧。
    std::vector<uint8_t> batch;
    for (int i = 0; i < 4; ++i) batch.insert(batch.end(), max_frame.begin(), max_frame.end());
    link.Receive_From_ISR(batch.data(), 512); link.Process();
    assert(callbacks == 8);
    link.Receive_From_ISR(batch.data() + 512, batch.size() - 512); link.Process();
    assert(callbacks == 9);
    link.Receive_From_ISR(data.data(), 8); link.Process();
    tick = 101; link.Process();
    link.Receive_From_ISR(data.data(), data.size()); link.Process();
    assert(callbacks == 10);
    std::vector<uint8_t> overflow(2048, 0);
    link.Receive_From_ISR(overflow.data(), overflow.size()); link.Process();
    link.Receive_From_ISR(data.data(), data.size()); link.Process();
    assert(link.Get_Overflow_Count() == 1 && callbacks == 11);
    // 多次循环越过队列末尾。
    for (int i = 0; i < 300; ++i)
    {
        link.Receive_From_ISR(data.data(), data.size()); link.Process();
    }
    assert(callbacks == 311);
    tick += 500; assert(!link.Is_Online());
    tick = 0xfffffff0;
    link.Receive_From_ISR(data.data(), data.size()); link.Process();
    tick = 20; assert(link.Is_Online()); // tick 回绕

    assert(link.Send(0x10, nullptr, 1) == HAL_ERROR);
    assert(link.Send(0x10, data.data(), 129) == HAL_ERROR);
    assert(link.Send(0x10, data.data(), data.size()) == HAL_OK);
    const auto saved = sent;
    assert(link.Send(0x01, nullptr, 0) == HAL_BUSY);
    assert(std::memcmp(active_tx, saved.data(), saved.size()) == 0);
    uart.gState = HAL_UART_STATE_READY;
    assert(link.Send(0x01, nullptr, 0) == HAL_OK && sent[6] == 1);

    tick = 0; Communication_Init();
#if !COMMUNICATION_NAVIGATION_ENABLE
    assert(board_rx && !nav_rx); // VOFA owns UART7; generic task must not register it.
    Struct_Navigation_Command disabled_command;
    assert(!Navigation_Get_Command(disabled_command));
    assert(Navigation_Send_Feedback(1, 2, 3, 4, 5, 6) == HAL_ERROR);
    tick = 200; Communication_Task();
    assert(huart7.gState == HAL_UART_STATE_READY); // no navigation heartbeat on VOFA port
    puts("communication tests passed (navigation disabled, UART7 reserved for VOFA)");
    return 0;
#endif
    assert(board_rx && nav_rx);
    Struct_Navigation_Command command;
    assert(!Navigation_Get_Command(command) && !command.enable);
    float velocity[] = {0.2f, -0.1f, 0.3f};
    std::vector<uint8_t> payload(13);
    std::memcpy(payload.data(), velocity, 12); payload[12] = 1;
    auto nav = frame(0x20, payload);
    nav_rx(nav.data(), 5); Communication_Task();
    assert(!Navigation_Get_Command(command));
    nav_rx(nav.data() + 5, nav.size() - 5); Communication_Task();
    assert(Navigation_Get_Command(command) && command.enable && command.vx_m_s == velocity[0]);
    assert(command.sequence == 7);
    velocity[0] = std::numeric_limits<float>::quiet_NaN();
    std::memcpy(payload.data(), velocity, 12);
    auto nan = frame(0x20, payload);
    tick = 150; nav_rx(nan.data(), nan.size()); Communication_Task();
    assert(Navigation_Get_Command(command) && command.received_ms == 0);
    huart7.gState = HAL_UART_STATE_READY;
    auto heartbeat = frame(0x01);
    nav_rx(heartbeat.data(), heartbeat.size()); Communication_Task();
    tick = 200;
    assert(!Navigation_Get_Command(command) && !command.enable && command.vx_m_s == 0);
    payload[12] = 2;
    auto invalid_enable = frame(0x20, payload);
    nav_rx(invalid_enable.data(), invalid_enable.size()); Communication_Task();
    assert(!Navigation_Get_Command(command));
    std::memcpy(payload.data(), velocity + 1, 4); // 恢复为有限速度
    payload[12] = 0;
    auto stop = frame(0x20, payload);
    nav_rx(stop.data(), stop.size()); Communication_Task();
    assert(Navigation_Get_Command(command) && !command.enable);
    huart7.gState = HAL_UART_STATE_READY;
    assert(Navigation_Send_Feedback(1, 2, 3, 4, 5, 6) == HAL_OK);
    assert(sent[3] == 0x21 && sent[4] == 24 && sent.size() == 34);
    puts("communication tests passed");
}
