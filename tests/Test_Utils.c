/**
 * @file Test_Utils.c
 * @brief Implementation of unified test timing utilities.
 */

#include "Test_Utils.h"
#include "HAL_io.h"
#include "BatteryPack.h"

/**
 * @brief Internal function: Advance physics and time WITHOUT FSM polling.
 *
 * This is called by public timing functions. Separated to allow tests to control
 * when FSM is called independently from physics advancement.
 */
static void _PhysicsStep_Ms(uint32_t elapsed_ms) {
    /* Advance HAL clock (also advances CircuitSim via HAL_Delay_us) */
    HAL_Delay_ms(elapsed_ms);
}

void Test_Timestep_Ms(uint32_t elapsed_ms) {
    _PhysicsStep_Ms(elapsed_ms);
}

void Test_Timestep_WithFSM_Ms(uint32_t elapsed_ms) {
    _PhysicsStep_Ms(elapsed_ms);

    /* Poll FSM after physics advances to ensure ADC reads see updated ngspice state */
    BatteryPack_FSM();
}

void Test_SimulationLoop_Ms(uint32_t duration_ms, uint32_t dt_ms) {
    if (dt_ms == 0) {
        return;  /* Prevent infinite loop */
    }

    uint32_t elapsed = 0;
    while (elapsed < duration_ms) {
        uint32_t step = (duration_ms - elapsed) > dt_ms ? dt_ms : (duration_ms - elapsed);
        Test_Timestep_Ms(step);
        elapsed += step;
    }
}

void Test_SimulationLoopWithFSM_Ms(uint32_t duration_ms, uint32_t dt_ms) {
    if (dt_ms == 0) {
        return;  /* Prevent infinite loop */
    }

    uint32_t elapsed = 0;
    while (elapsed < duration_ms) {
        uint32_t step = (duration_ms - elapsed) > dt_ms ? dt_ms : (duration_ms - elapsed);
        Test_Timestep_WithFSM_Ms(step);
        elapsed += step;
    }
}
