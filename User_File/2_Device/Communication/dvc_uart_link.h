#ifndef DVC_UART_LINK_H
#define DVC_UART_LINK_H

#include "1_Middleware/Driver/UART/drv_uart.h"

// 两端共同遵守的暂定协议，详见同目录 README.md。
enum Enum_UART_Link_Message : uint8_t
{
    UART_LINK_HEARTBEAT = 0x01,
    UART_LINK_BOARD_DATA = 0x10,
    UART_LINK_NAV_VELOCITY = 0x20,
    UART_LINK_NAV_FEEDBACK = 0x21,
};

using UART_Link_Frame_Callback = void (*)(uint8_t type, uint16_t sequence,
                                         const uint8_t *payload, uint16_t length);

class Class_UART_Link
{
public:
    static constexpr uint16_t MAX_PAYLOAD = 128;
    void Init(UART_HandleTypeDef *uart, UART_Link_Frame_Callback callback);
    // 只在串口中断调用：复制 DMA 数据，耗时业务放到 Process 中执行。
    void Receive_From_ISR(const uint8_t *data, uint16_t length);
    // 仅在前台 Task_Loop 调用。
    void Process();
    // 仅在前台调用。内部保存发送数据，DMA 完成前不会覆盖。
    // HAL_BUSY 表示本次未发送，调用者下次重试；HAL_ERROR 表示参数错误。
    HAL_StatusTypeDef Send(uint8_t type, const uint8_t *payload, uint16_t length);
    bool Is_Online(uint32_t timeout_ms = 500) const;
    uint32_t Get_Rx_Frame_Count() const { return rx_frames_; }
    uint32_t Get_CRC_Error_Count() const { return crc_errors_; }
    uint32_t Get_Overflow_Count() const { return overflows_; }

private:
    static constexpr uint16_t FRAME_SIZE = MAX_PAYLOAD + 10;
    static constexpr uint16_t QUEUE_SIZE = 2048;
    UART_HandleTypeDef *uart_ = nullptr;
    UART_Link_Frame_Callback callback_ = nullptr;
    uint8_t queue_[QUEUE_SIZE] = {};
    volatile uint16_t head_ = 0;
    volatile uint16_t tail_ = 0;
    volatile bool overflow_pending_ = false;
    volatile uint32_t overflows_ = 0;
    uint8_t frame_[FRAME_SIZE] = {};
    uint16_t used_ = 0;
    uint8_t tx_[FRAME_SIZE] = {};
    uint16_t tx_sequence_ = 0;
    uint32_t rx_frames_ = 0;
    uint32_t crc_errors_ = 0;
    uint32_t last_rx_ms_ = 0;
    uint32_t last_byte_ms_ = 0;
    void Parse_Byte(uint8_t byte);
    void Discard(uint16_t length);
    static uint16_t CRC16(const uint8_t *data, uint16_t length);
};

#endif
