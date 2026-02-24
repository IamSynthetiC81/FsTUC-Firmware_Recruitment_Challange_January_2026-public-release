#include "Clock.h"

static bool g_clock_initialized = false;
static uint64_t g_clock_start_ns = 0U;
static uint64_t g_clock_sim_ns = 0U;

static void Clock_EnsureInit(void) {
    if (!g_clock_initialized) {
        g_clock_sim_ns = 0U;
        g_clock_start_ns = 0U;
        g_clock_initialized = true;
    }
}

void Clock_Init(void) {
    g_clock_sim_ns = 0U;
    g_clock_start_ns = 0U;
    g_clock_initialized = true;
}

bool Clock_IsInitialized(void) {
    return g_clock_initialized;
}

uint64_t Clock_GetTimeNs(void) {
    Clock_EnsureInit();
    return g_clock_sim_ns;
}

uint64_t Clock_GetTimeUs(void) {
    return Clock_GetTimeNs() / 1000ULL;
}

uint64_t Clock_GetTimeMs(void) {
    return Clock_GetTimeUs() / 1000ULL;
}

uint64_t Clock_GetSimTimeNs(void) {
    Clock_EnsureInit();
    return g_clock_sim_ns - g_clock_start_ns;
}

uint64_t Clock_GetSimTimeUs(void) {
    return Clock_GetSimTimeNs() / 1000ULL;
}

uint32_t Clock_GetCpuHz(void) {
    return CLOCK_CPU_HZ;
}

uint32_t Clock_GetCyclesPerUs(void) {
    return CLOCK_CPU_HZ / 1000000U;
}

uint64_t Clock_GetCycles(void) {
    uint64_t elapsed_us = Clock_GetSimTimeUs();
    return (elapsed_us * (uint64_t)CLOCK_CPU_HZ) / 1000000ULL;
}

uint64_t Clock_GetCycleRuntime(void) {
    return Clock_GetCycles();
}

void Clock_DelayUs(uint32_t microseconds) {
    Clock_EnsureInit();
    g_clock_sim_ns += (uint64_t)microseconds * 1000ULL;
}

void Clock_DelayMs(uint32_t milliseconds) {
    Clock_DelayUs(milliseconds * 1000U);
}
