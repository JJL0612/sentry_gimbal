#include "tsk_chassis_link.h"
#include "tsk_dm_imu.h"
#include "1_Middleware/Driver/UART/drv_uart.h"

chassis_link::Link Chassis_Link;
bool chassis_link_boot_valid = false;
volatile unsigned chassis_link_rx_overflows = 0;
namespace {
std::uint8_t pending[512]{};
std::size_t pending_size = 0;
bool overflow = false;
std::uint8_t tx[chassis_link::batch_size]{}; // DMA must retain ownership until gState is READY.
std::uint32_t last_tx_ms = 0;
void receive(std::uint8_t *data, std::uint16_t size) {
    if (!init_finished) return;
    if (overflow) return;
    if (size > sizeof(pending) - pending_size) {
        pending_size = 0; overflow = true; ++chassis_link_rx_overflows;
        return;
    }
    memcpy(pending + pending_size, data, size);
    pending_size += size;
}
std::uint64_t newBootId() {
    // HSI48 is already enabled by SystemClock_Config for USB. Use hardware entropy
    // so a gimbal reboot cannot be confused with old packet sequence numbers.
    RCC_PeriphCLKInitTypeDef clock{};
    clock.PeriphClockSelection = RCC_PERIPHCLK_RNG;
    clock.RngClockSelection = RCC_RNGCLKSOURCE_HSI48;
    if (HAL_RCCEx_PeriphCLKConfig(&clock) != HAL_OK) return 0;
    __HAL_RCC_RNG_CLK_ENABLE();
    RNG_HandleTypeDef rng{};
    rng.Instance = RNG;
    std::uint32_t high = 0, low = 0;
    if (HAL_RNG_Init(&rng) != HAL_OK) return 0;
    const bool ok = HAL_RNG_GenerateRandomNumber(&rng, &high) == HAL_OK &&
                    HAL_RNG_GenerateRandomNumber(&rng, &low) == HAL_OK;
    HAL_RNG_DeInit(&rng);
    __HAL_RCC_RNG_CLK_DISABLE();
    return ok ? ((std::uint64_t(high) << 32) | low | 1ULL) : 0;
}
}
void Chassis_Link_Init() {
    const auto boot = newBootId();
    chassis_link_boot_valid = boot != 0;
    Chassis_Link.init(boot);
    UART_Init(&huart10, receive);
}
void Chassis_Link_Tick(const chassis_link::Input &input) {
    std::uint8_t bytes[sizeof(pending)];
    const auto mask = __get_PRIMASK();
    __disable_irq();
    const auto n = pending_size;
    memcpy(bytes, pending, n);
    const bool lost = overflow;
    pending_size = 0; overflow = false;
    __set_PRIMASK(mask);
    const auto now = HAL_GetTick();
    if (lost) Chassis_Link.discardPartial();
    Chassis_Link.consume(bytes, n, now);
    auto enriched = input;
    Dm_Imu_FillInput(enriched);
    Chassis_Link.update(enriched, now);
    if (std::uint32_t(now - last_tx_ms) >= 20 && huart10.gState == HAL_UART_STATE_READY) {
        const auto size = Chassis_Link.encode(tx, sizeof(tx), now);
        if (size && UART_Transmit_Data(&huart10, tx, size) == HAL_OK) last_tx_ms = now;
    }
}
