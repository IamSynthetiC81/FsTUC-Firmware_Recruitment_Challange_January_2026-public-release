/**
 * @file Test_Utils.h
 * @brief Unified test timing and simulation utilities.
 *
 * This module consolidates all timing operations in tests to prevent double-advancement
 * and ensure consistent synchronization between HAL clock, hardware physics simulation,
 * and battery physics simulation.
 *
 * Test code controls FSM polling independently from physics advancement, so timing
 * functions are available in two variants:
 * - Test_Timestep_Ms/Test_SimulationLoop_Ms: Physics only (no FSM)
 * - Test_Timestep_WithFSM_Ms/Test_SimulationLoopWithFSM_Ms: Physics + FSM polling
 */

#ifndef TEST_UTILS_H
#define TEST_UTILS_H

#include <stdint.h>

/**
 * @brief Advance physics and clocks by the specified milliseconds (no FSM).
 *
 * Advances:
 * - HAL clock by elapsed_ms
 * - Hardware physics (contactors) by elapsed_ms
 * - Battery frontend physics by elapsed_ms (converted to seconds)
 *
 * FSM is NOT polled. Use this when tests manage FSM calling separately.
 *
 * @param elapsed_ms Number of milliseconds to advance
 */
void Test_Timestep_Ms(uint32_t elapsed_ms);

/**
 * @brief Advance physics, clocks, and poll FSM by the specified milliseconds.
 *
 * Same as Test_Timestep_Ms but also polls FSM state machine once.
 * Useful for autonomous simulation loops.
 *
 * @param elapsed_ms Number of milliseconds to advance
 */
void Test_Timestep_WithFSM_Ms(uint32_t elapsed_ms);

/**
 * @brief Advance simulation for a duration with fixed timesteps (physics only).
 *
 * Repeatedly calls Test_Timestep_Ms() with fixed dt_ms timesteps until duration
 * is exhausted. Use when tests manage FSM polling separately.
 *
 * @param duration_ms Total duration in milliseconds
 * @param dt_ms Individual timestep in milliseconds (should be positive)
 */
void Test_SimulationLoop_Ms(uint32_t duration_ms, uint32_t dt_ms);

/**
 * @brief Advance simulation for a duration with FSM polling (with FSM).
 *
 * Repeatedly calls Test_Timestep_WithFSM_Ms() with fixed dt_ms timesteps.
 * FSM is polled once per timestep.
 *
 * @param duration_ms Total duration in milliseconds
 * @param dt_ms Individual timestep in milliseconds (should be positive)
 */
void Test_SimulationLoopWithFSM_Ms(uint32_t duration_ms, uint32_t dt_ms);

#endif // TEST_UTILS_H
