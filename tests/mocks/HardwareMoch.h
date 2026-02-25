/**
 * @file HardwareMoch.h
 * @brief Public API for the HardwareMoch support library.
 */

#ifndef HARDWARE_MOCH_H
#define HARDWARE_MOCH_H

#include <stdint.h>
#include "HAL_io_mock.h"

/**
 * @brief Advance physics and clocks by the specified milliseconds (no FSM).
 *
 * @param elapsed_ms Number of milliseconds to advance
 */
void Test_Timestep_Ms(uint32_t elapsed_ms);

/**
 * @brief Advance physics, clocks, and poll FSM by the specified milliseconds.
 *
 * @param elapsed_ms Number of milliseconds to advance
 */
void Test_Timestep_WithFSM_Ms(uint32_t elapsed_ms);

/**
 * @brief Advance simulation for a duration with fixed timesteps (physics only).
 *
 * @param duration_ms Total duration in milliseconds
 * @param dt_ms Individual timestep in milliseconds
 */
void Test_SimulationLoop_Ms(uint32_t duration_ms, uint32_t dt_ms);

/**
 * @brief Advance simulation for a duration with FSM polling.
 *
 * @param duration_ms Total duration in milliseconds
 * @param dt_ms Individual timestep in milliseconds
 */
void Test_SimulationLoopWithFSM_Ms(uint32_t duration_ms, uint32_t dt_ms);

#endif /* HARDWARE_MOCH_H */
