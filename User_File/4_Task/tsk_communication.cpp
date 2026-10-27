#include "tsk_communication.h"
#include <cmath>

Class_UART_Link DM_Board_Link;
Class_UART_Link Navigation_Link;

namespace
{
Struct_Navigation_Command navigation_command;
bool navigation_received = false;
uint32_t board_heartbeat_ms = 0;
uint32_t navigation_heartbeat_ms = 0;

void UART1_Callback(uint8_t *buffer, uint16_t length)
{
    DM_Board_Link.Receive_From_ISR(buffer, length);
}
void UART7_Callback(uint8_t *buffer, uint16_t length)
{
    Navigation_Link.Receive_From_ISR(buffer, length);
}

void Board_Frame_Callback(uint8_t type, uint16_t sequence,
                          const uint8_t *payload, uint16_t length)
{
    if (type == UART_LINK_BOARD_DATA)
    {
        // TODO: 在这里校验板间业务长度、解码并缓存另一块 DM 板的数据。
        // payload 只在本次回调期间有效，需要保存时复制到自己的结构体。
        (void)sequence;
        (void)payload;
        (void)length;
    }
}

void Navigation_Frame_Callback(uint8_t type, uint16_t sequence,
                               const uint8_t *payload, uint16_t length)
{
    if (type != UART_LINK_NAV_VELOCITY || length != 13 || payload[12] > 1) return;
    Struct_Navigation_Command next;
    // STM32 与小电脑协议采用小端 IEEE754 float32，禁止直接发送带填充的结构体。
    memcpy(&next.vx_m_s, payload, 4);
    memcpy(&next.vy_m_s, payload + 4, 4);
    memcpy(&next.wz_rad_s, payload + 8, 4);
    if (!std::isfinite(next.vx_m_s) || !std::isfinite(next.vy_m_s) ||
        !std::isfinite(next.wz_rad_s)) return;
    next.enable = payload[12] != 0;
    next.sequence = sequence;
    next.received_ms = HAL_GetTick();
    // 前台写、控制中断读：发布完整快照，防止读到一半更新的速度。
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    navigation_command = next;
    navigation_received = true;
    __set_PRIMASK(primask);
}
}

void Communication_Init()
{
    navigation_command = {};
    navigation_received = false;
    board_heartbeat_ms = navigation_heartbeat_ms = HAL_GetTick();
    DM_Board_Link.Init(&huart1, Board_Frame_Callback);
    if (COMMUNICATION_NAVIGATION_ENABLE)
        Navigation_Link.Init(&huart7, Navigation_Frame_Callback);
    UART_Init(&huart1, UART1_Callback);
    if (COMMUNICATION_NAVIGATION_ENABLE)
        UART_Init(&huart7, UART7_Callback);
}

void Communication_Task()
{
    DM_Board_Link.Process();
    if (COMMUNICATION_NAVIGATION_ENABLE) Navigation_Link.Process();
    const uint32_t now = HAL_GetTick();
    // 心跳与业务共用发送接口，DMA 忙时留到下一次循环重试。
    if (now - board_heartbeat_ms >= 100 &&
        DM_Board_Link.Send(UART_LINK_HEARTBEAT, nullptr, 0) == HAL_OK)
        board_heartbeat_ms = now;
    if (COMMUNICATION_NAVIGATION_ENABLE && now - navigation_heartbeat_ms >= 100 &&
        Navigation_Link.Send(UART_LINK_HEARTBEAT, nullptr, 0) == HAL_OK)
        navigation_heartbeat_ms = now;

    // TODO: 在这里按所需周期调用 DM_Board_Send / Navigation_Send_Feedback。
    // 导航反馈需填入真实里程计；收到导航速度后在控制层结合遥控使能与限速执行。
}

bool Navigation_Get_Command(Struct_Navigation_Command &command)
{
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    command = navigation_command;
    const bool received = navigation_received;
    __set_PRIMASK(primask);
    if (!COMMUNICATION_NAVIGATION_ENABLE || !received || HAL_GetTick() - command.received_ms >= 200)
    {
        command = {};
        return false;
    }
    return true;
}

HAL_StatusTypeDef Navigation_Send_Feedback(float x, float y, float yaw,
                                         float vx, float vy, float wz)
{
    if (!COMMUNICATION_NAVIGATION_ENABLE) return HAL_ERROR;
    const float values[6] = {x, y, yaw, vx, vy, wz};
    for (float value : values) if (!std::isfinite(value)) return HAL_ERROR;
    uint8_t payload[24];
    static_assert(sizeof(float) == 4, "Protocol requires float32");
    memcpy(payload, values, sizeof(payload));
    return Navigation_Link.Send(UART_LINK_NAV_FEEDBACK, payload, sizeof(payload));
}

HAL_StatusTypeDef DM_Board_Send(const uint8_t *payload, uint16_t length)
{
    return DM_Board_Link.Send(UART_LINK_BOARD_DATA, payload, length);
}
