#ifndef TSK_COMMUNICATION_H
#define TSK_COMMUNICATION_H

#include "2_Device/Communication/dvc_uart_link.h"

// UART7 belongs to VOFA; navigation is disabled in this firmware.
// USART1 retains the generic link; USART10 exclusively runs the chassis protocol.
#ifndef COMMUNICATION_NAVIGATION_ENABLE
#define COMMUNICATION_NAVIGATION_ENABLE 0
#endif
extern Class_UART_Link DM_Board_Link;
extern Class_UART_Link Navigation_Link;

struct Struct_Navigation_Command
{
    float vx_m_s = 0.0f;       // 机体坐标系，x 向前
    float vy_m_s = 0.0f;       // 机体坐标系，y 向左
    float wz_rad_s = 0.0f;     // 逆时针为正
    bool enable = false;
    uint16_t sequence = 0;
    uint32_t received_ms = 0;
};

void Communication_Init();
void Communication_Task();
// 可在控制中断读取快照；无有效指令或超过 200ms，返回 false 并清零输出。
// 返回 true 只表示指令新鲜，执行还应检查 command.enable 和遥控使能。
bool Navigation_Get_Command(Struct_Navigation_Command &command);
// 仅前台调用；实际里程计就绪后由用户周期调用，单位 m、rad、m/s、rad/s。
HAL_StatusTypeDef Navigation_Send_Feedback(float x, float y, float yaw,
                                         float vx, float vy, float wz);
// 仅前台调用；板间业务数据内容由两块板共同约定。
HAL_StatusTypeDef DM_Board_Send(const uint8_t *payload, uint16_t length);

#endif
