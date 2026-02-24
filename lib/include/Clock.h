#ifndef CLOCK_H
#define CLOCK_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define CLOCK_CPU_HZ 48000000U

/**
 * @brief Initialize the clock module.
 */
void Clock_Init(void);

/**
 * @brief Check if the clock module is initialized.
 */
bool Clock_IsInitialized(void);

/**
 * @brief Get simulated monotonic time in nanoseconds.
 */
uint64_t Clock_GetTimeNs(void);

/**
 * @brief Get simulated monotonic time in microseconds.
 */
uint64_t Clock_GetTimeUs(void);

/**
 * @brief Get simulated monotonic time in milliseconds.
 */
uint64_t Clock_GetTimeMs(void);

/**
 * @brief Get simulation time in nanoseconds since clock init.
 */
uint64_t Clock_GetSimTimeNs(void);

/**
 * @brief Get simulation time in microseconds since clock init.
 */
uint64_t Clock_GetSimTimeUs(void);

/**
 * @brief Get configured CPU frequency in Hz.
 */
uint32_t Clock_GetCpuHz(void);

/**
 * @brief Get cycles per microsecond.
 */
uint32_t Clock_GetCyclesPerUs(void);

/**
 * @brief Get cycles since clock init.
 */
uint64_t Clock_GetCycles(void);

/**
 * @brief Get cycle runtime since clock init.
 */
uint64_t Clock_GetCycleRuntime(void);

/**
 * @brief Advance the simulated time by the specified microseconds.
 */
void Clock_DelayUs(uint32_t microseconds);

/**
 * @brief Advance the simulated time by the specified milliseconds.
 */
void Clock_DelayMs(uint32_t milliseconds);

#ifdef __cplusplus
}
#endif

#endif /* CLOCK_H */
