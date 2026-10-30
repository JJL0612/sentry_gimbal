#ifndef TSK_COMMUNICATION_H
#define TSK_COMMUNICATION_H
#include "1_Middleware/Driver/UART/drv_uart.h"
#include "2_Device/Communication/dvc_chassis_link.h"

// Board connector labelled USART3 uses PA9(TX)/PA10(RX): MCU USART1 / huart1.
// Navigation: 921600 8N1, ordinary UART. Old generic UART1 link is disabled.
struct Struct_Navigation_Command
{
    float vx_m_s = 0.0f;   // Raw navigation velocity; translation rotated in Apply_To_Chassis.
    float vy_m_s = 0.0f;
    float wz_rad_s = 0.0f;
    bool enable = false;  // Local freshness flag; no enable field on the wire.
    uint16_t sequence = 0; // Local receive counter; no sequence field on the wire.
    uint32_t received_ms = 0;
};
struct Struct_Navigation_Statistics
{
    uint32_t rx_frames = 0, checksum_errors = 0, overflows = 0;
};
void Communication_Init();
void Communication_Task();
// Safe to read from control ISR. At >=200ms since reception, clears output and returns false.
bool Navigation_Get_Command(Struct_Navigation_Command &command);
Struct_Navigation_Statistics Navigation_Get_Statistics();
// MID: navigation (stale/offline => zero); UP: existing remote velocities.
void Navigation_Apply_To_Chassis(chassis_link::Input &input);
// Optional foreground TX using the SAME 9-byte velocity format. No automatic TX:
// the supplied protocol does not define odometry feedback or heartbeat messages.
// Finite values rounded to 0.001 and saturated to +/-32.767. DMA busy => retry later.
HAL_StatusTypeDef Navigation_Send_Velocity(float vx, float vy, float wz);
#endif
