#pragma once
#include <cstdint>
#include <cstring>
enum HAL_StatusTypeDef { HAL_OK, HAL_ERROR, HAL_BUSY, HAL_TIMEOUT };
constexpr uint32_t HAL_UART_STATE_READY = 0;
constexpr uint32_t HAL_UART_STATE_BUSY_TX = 1;
struct UART_HandleTypeDef { uint32_t gState = HAL_UART_STATE_READY; };
using UART_Callback = void (*)(uint8_t *, uint16_t);
extern UART_HandleTypeDef huart1, huart3, huart7, huart10;
uint32_t HAL_GetTick();
HAL_StatusTypeDef HAL_UART_Transmit_DMA(UART_HandleTypeDef *, uint8_t *, uint16_t);
void UART_Init(UART_HandleTypeDef *, UART_Callback);
inline uint32_t __get_PRIMASK() { return 0; }
inline void __disable_irq() {}
inline void __set_PRIMASK(uint32_t) {}
inline void __DMB() {}
