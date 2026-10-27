#pragma once
#include "2_Device/Communication/dvc_chassis_link.h"

// RX ISR only queues bytes. TIM7 drains and owns the protocol state and TX buffer.
void Chassis_Link_Init();
void Chassis_Link_Tick(const chassis_link::Input &input);
extern chassis_link::Link Chassis_Link;
extern bool chassis_link_boot_valid;
extern volatile unsigned chassis_link_rx_overflows;
