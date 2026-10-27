#include "dvc_uart_link.h"

namespace
{
uint16_t Read_U16(const uint8_t *data)
{
    return static_cast<uint16_t>(data[0] | (static_cast<uint16_t>(data[1]) << 8));
}
void Write_U16(uint8_t *data, uint16_t value)
{
    data[0] = static_cast<uint8_t>(value);
    data[1] = static_cast<uint8_t>(value >> 8);
}
}

void Class_UART_Link::Init(UART_HandleTypeDef *uart, UART_Link_Frame_Callback callback)
{
    uart_ = uart;
    callback_ = callback;
    head_ = tail_ = used_ = tx_sequence_ = 0;
    overflow_pending_ = false;
    overflows_ = rx_frames_ = crc_errors_ = 0;
    last_rx_ms_ = last_byte_ms_ = HAL_GetTick();
}

void Class_UART_Link::Receive_From_ISR(const uint8_t *data, uint16_t length)
{
    if (data == nullptr || length == 0) return;
    const uint16_t free = (tail_ + QUEUE_SIZE - head_ - 1) % QUEUE_SIZE;
    if (overflow_pending_ || length > free)
    {
        ++overflows_;
        overflow_pending_ = true;
        return;
    }
    uint16_t next = head_;
    for (uint16_t i = 0; i < length; ++i)
    {
        queue_[next] = data[i];
        next = (next + 1) % QUEUE_SIZE;
    }
    __DMB();
    head_ = next;
}

void Class_UART_Link::Process()
{
    // 半帧长时间没有后续数据则丢弃，防止错误长度永久阻塞后续帧。
    if (used_ != 0 && HAL_GetTick() - last_byte_ms_ > 100) used_ = 0;
    // 每次最多处理一个队列容量，避免持续输入占满前台。
    for (uint16_t count = 0; count < QUEUE_SIZE; ++count)
    {
        const uint32_t primask = __get_PRIMASK();
        __disable_irq();
        if (overflow_pending_)
        {
            tail_ = head_;
            used_ = 0;
            overflow_pending_ = false;
        }
        if (tail_ == head_)
        {
            __set_PRIMASK(primask);
            break;
        }
        const uint8_t byte = queue_[tail_];
        tail_ = (tail_ + 1) % QUEUE_SIZE;
        __set_PRIMASK(primask);
        last_byte_ms_ = HAL_GetTick();
        Parse_Byte(byte);
    }
}

void Class_UART_Link::Discard(uint16_t length)
{
    used_ -= length;
    memmove(frame_, frame_ + length, used_);
}

void Class_UART_Link::Parse_Byte(uint8_t byte)
{
    frame_[used_++] = byte;
    while (used_ != 0)
    {
        if (frame_[0] != 0xA5) { Discard(1); continue; }
        if (used_ < 2) return;
        if (frame_[1] != 0x5A) { Discard(1); continue; }
        if (used_ < 8) return;
        const uint16_t length = Read_U16(frame_ + 4);
        if (frame_[2] != 1 || length > MAX_PAYLOAD) { Discard(1); continue; }
        const uint16_t total = length + 10;
        if (used_ < total) return;
        if (CRC16(frame_, total - 2) != Read_U16(frame_ + total - 2))
        {
            ++crc_errors_;
            Discard(1);
            continue;
        }
        ++rx_frames_;
        last_rx_ms_ = HAL_GetTick();
        if (callback_ != nullptr)
            callback_(frame_[3], Read_U16(frame_ + 6), frame_ + 8, length);
        Discard(total);
    }
}

uint16_t Class_UART_Link::CRC16(const uint8_t *data, uint16_t length)
{
    // CRC-16/CCITT-FALSE: poly=0x1021, init=0xFFFF, xorout=0, 不反射。
    uint16_t crc = 0xFFFF;
    for (uint16_t i = 0; i < length; ++i)
    {
        crc ^= static_cast<uint16_t>(data[i]) << 8;
        for (uint8_t bit = 0; bit < 8; ++bit)
            crc = (crc & 0x8000) ? static_cast<uint16_t>((crc << 1) ^ 0x1021)
                                 : static_cast<uint16_t>(crc << 1);
    }
    return crc;
}

HAL_StatusTypeDef Class_UART_Link::Send(uint8_t type, const uint8_t *payload, uint16_t length)
{
    if (uart_ == nullptr || length > MAX_PAYLOAD || (length != 0 && payload == nullptr))
        return HAL_ERROR;
    if (uart_->gState != HAL_UART_STATE_READY) return HAL_BUSY;
    tx_[0] = 0xA5;
    tx_[1] = 0x5A;
    tx_[2] = 1;
    tx_[3] = type;
    Write_U16(tx_ + 4, length);
    Write_U16(tx_ + 6, tx_sequence_);
    if (length != 0) memcpy(tx_ + 8, payload, length);
    Write_U16(tx_ + length + 8, CRC16(tx_, length + 8));
    const HAL_StatusTypeDef status = HAL_UART_Transmit_DMA(uart_, tx_, length + 10);
    if (status == HAL_OK) ++tx_sequence_;
    return status;
}

bool Class_UART_Link::Is_Online(uint32_t timeout_ms) const
{
    return rx_frames_ != 0 && HAL_GetTick() - last_rx_ms_ < timeout_ms;
}
