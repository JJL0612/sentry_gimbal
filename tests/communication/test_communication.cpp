#include "4_Task/tsk_communication.h"
#include "2_Device/Communication/dvc_uart_link.h"
#include <cmath>
#include <cassert>
#include <cstdio>
#include <limits>
#include <vector>

UART_HandleTypeDef huart1, huart3, huart7, huart10;
static uint32_t tick;
static UART_Callback nav_rx;
static uint8_t *active_tx;
static std::vector<uint8_t> sent;
uint32_t HAL_GetTick() { return tick; }
void UART_Init(UART_HandleTypeDef *uart, UART_Callback callback)
{
    assert(uart == &huart1); // USART10 belongs to the chassis link.
    nav_rx = callback;
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
    assert(nav_rx); // UART7 reserved for VOFA; USART10 for chassis.
    Struct_Navigation_Command command;
    assert(!Navigation_Get_Command(command));
    // vx=1.000, vy=-0.500, wz=0.250, XOR=4F (independent fixed vector).
    std::vector<uint8_t> nav{0xAA,0x06,0xE8,0x03,0x0C,0xFE,0xFA,0x00,0x4F};
    uint8_t sum = 0; for (unsigned i=0;i<8;++i) sum ^= nav[i];
    assert(sum == nav[8]);
    for (unsigned i=0; i<nav.size(); ++i) {
        nav_rx(&nav[i],1); Communication_Task();
        assert(Navigation_Get_Command(command) == (i == 8));
    }
    assert(command.enable && command.vx_m_s == 1.0f);
    assert(command.vy_m_s == -0.5f && std::fabs(command.wz_rad_s - 0.25f) < 1e-6f);
    assert(command.sequence == 1);
    chassis_link::Input input;
    input.remote_online = true; input.left_switch = 0;
    input.vx = input.vy = input.wz = 9;
    Navigation_Apply_To_Chassis(input);
    assert(input.vx == -0.5f && input.vy == -1 && std::fabs(input.wz-0.25f)<1e-6f);
    input.left_switch = 1; input.vx = 0.1f; input.wz = 0.8;
    Navigation_Apply_To_Chassis(input); assert(input.vx == 0.1f && input.wz == 0.8f);
    input.left_switch = 0; input.remote_online = false;
    Navigation_Apply_To_Chassis(input); assert(input.vx == 0 && input.vy == 0 && input.wz == 0);
    input.remote_online = true;
    // Bad checksum, noise, bad length, then two frames in one callback.
    auto burst = nav; burst.back() ^= 1;
    burst.insert(burst.end(), {0x11,0xAA,0x07});
    burst.insert(burst.end(), nav.begin(),nav.end());
    burst.insert(burst.end(), nav.begin(),nav.end());
    tick = 50; nav_rx(burst.data(),burst.size()); Communication_Task();
    assert(Navigation_Get_Command(command) && command.sequence == 3);
    assert(Navigation_Get_Statistics().checksum_errors >= 1);
    tick = 249; assert(Navigation_Get_Command(command));
    tick = 250; assert(!Navigation_Get_Command(command) && command.vx_m_s == 0 && !command.enable);
    input.vx = input.vy = input.wz = 9;
    Navigation_Apply_To_Chassis(input); assert(input.vx == 0 && input.vy == 0 && input.wz == 0);
    // Foreground delay must not make stale bytes look fresh.
    tick = 300; nav_rx(nav.data(),nav.size());
    tick = 500; Communication_Task(); assert(!Navigation_Get_Command(command));
    // Partial frame must expire according to arrival time, even if processed together.
    tick = 600; nav_rx(nav.data(),4);
    tick = 701; nav_rx(nav.data()+4,5); Communication_Task();
    assert(!Navigation_Get_Command(command));
    nav_rx(nav.data(),nav.size()); Communication_Task(); assert(Navigation_Get_Command(command));
    // Overflow invalidates cached commands immediately and recovers on a new frame.
    std::vector<uint8_t> flood(2048,0xAA);
    nav_rx(flood.data(),flood.size()); assert(!Navigation_Get_Command(command));
    Communication_Task(); nav_rx(nav.data(),nav.size()); Communication_Task();
    assert(Navigation_Get_Command(command) && Navigation_Get_Statistics().overflows == 1);
    // Ring wrapping and DMA-sized bursts (512 bytes).
    for (int i=0;i<300;++i) { nav_rx(nav.data(),nav.size()); Communication_Task(); }
    std::vector<uint8_t> block(503,0); block.insert(block.end(),nav.begin(),nav.end());
    nav_rx(block.data(),block.size()); Communication_Task(); assert(Navigation_Get_Command(command));
    tick = 0xfffffff0; nav_rx(nav.data(),nav.size()); Communication_Task();
    tick = 20; assert(Navigation_Get_Command(command));
    tick = 184; assert(!Navigation_Get_Command(command));
    // Axis regression: compensate the measured 90-degree physical offset.
    uint8_t forward[] = {0xAA,0x06,0xE8,0x03,0,0,0,0,0x47};
    uint8_t left[] = {0xAA,0x06,0,0,0xE8,0x03,0,0,0x47};
    input.left_switch = 0; input.remote_online = true;
    nav_rx(forward, sizeof(forward)); Communication_Task(); Navigation_Apply_To_Chassis(input);
    assert(input.vx == 0 && input.vy == -1 && input.wz == 0);
    nav_rx(left, sizeof(left)); Communication_Task(); Navigation_Apply_To_Chassis(input);
    assert(input.vx == 1 && input.vy == 0 && input.wz == 0);
    huart1.gState = HAL_UART_STATE_READY;
    assert(Navigation_Send_Velocity(1,-0.5f,0.25f) == HAL_OK && sent == nav);
    const auto saved_nav = sent;
    assert(Navigation_Send_Velocity(9,8,7) == HAL_BUSY);
    assert(std::memcmp(active_tx,saved_nav.data(),saved_nav.size()) == 0);
    huart1.gState = HAL_UART_STATE_READY;
    assert(Navigation_Send_Velocity(std::numeric_limits<float>::infinity(),0,0) == HAL_ERROR);
    assert(Navigation_Send_Velocity(100,-100,0) == HAL_OK);
    assert(sent[2] == 0xFF && sent[3] == 0x7F && sent[4] == 1 && sent[5] == 0x80);
    huart1.gState = HAL_UART_STATE_READY;
    Communication_Task(); assert(huart1.gState == HAL_UART_STATE_READY); // No heartbeat on navigation.
    tick += 1000; Communication_Task();
    assert(huart1.gState == HAL_UART_STATE_READY); // No generic heartbeat on navigation connector.
    assert(huart3.gState == HAL_UART_STATE_READY);
    assert(huart7.gState == HAL_UART_STATE_READY);
    puts("communication tests passed (generic parser + PA9/PA10 navigation on huart1)");
}
