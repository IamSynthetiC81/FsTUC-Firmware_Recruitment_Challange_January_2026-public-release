#include "FSM.h"
/* ============================================================================
 * FSM State Definitions
 * ========================================================================== */
FSM_State_t SAFE_STATE = {
    .air_p_state = OPEN,
    .air_n_state = OPEN,
    .precharge_state = OPEN,
    .discharge_state = OPEN,
    .state_id = SAFE_ID,
    .timestamp_ms = 0
};

FSM_State_t PRECHARGE_STATE = {
    .air_p_state = OPEN,
    .air_n_state = CLOSED,
    .precharge_state = CLOSED,
    .discharge_state = OPEN,
    .state_id = PRECHARGE_ID,
    .timestamp_ms = 0
};

FSM_State_t READY_TO_RACE_STATE = {
    .air_p_state = CLOSED,
    .air_n_state = CLOSED,
    .precharge_state = OPEN,
    .discharge_state = OPEN,
    .state_id = READY_TO_RACE_ID,
    .timestamp_ms = 0
};

FSM_State_t DISCHARGE_STATE = {
    .air_p_state = OPEN,
    .air_n_state = OPEN,
    .precharge_state = OPEN,
    .discharge_state = CLOSED,
    .state_id = DISCHARGE_ID,
    .timestamp_ms = 0
};

FSM_State_t FAULT_STATE = {
    .air_p_state = OPEN,
    .air_n_state = OPEN,
    .precharge_state = OPEN,
    .discharge_state = OPEN,
    .state_id = FAULT_ID,
    .timestamp_ms  = 0
};

static uint32_t _get_time_ms(void) {
    uint64_t ticks = HAL_GetTick();
    return (uint32_t)((ticks * 1000ULL) / CLOCK_CPU_HZ);
}

/* ============================================================================
 * Internal helpers
 * ========================================================================== */

bool _compare_contactor_states(const FSM_State_t* state1, const FSM_State_t* state2) {
    return (state1->air_p_state == state2->air_p_state) &&
           (state1->air_n_state == state2->air_n_state) &&
           (state1->precharge_state == state2->precharge_state) &&
           (state1->discharge_state == state2->discharge_state);
}

void _update_timestamp(FSM_State_t* state) {
    state->timestamp_ms = _get_time_ms();
}

void _reset_timestamp(FSM_State_t* state) {
    state->timestamp_ms = 0;
}

static bool FSM_SetLED(FSM_State_t* state) {
    switch (state->state_id) {
        case SAFE_ID:
            HAL_GPIO_WritePin(ready_to_race_indicator_pin, false);
            HAL_GPIO_WritePin(idle_indicator_pin, true);
            HAL_GPIO_WritePin(fault_indicator_pin, false);
            return EXIT_SUCCESS;
        case PRECHARGE_ID:
            HAL_GPIO_WritePin(ready_to_race_indicator_pin, false);
            HAL_GPIO_WritePin(idle_indicator_pin, false);
            HAL_GPIO_WritePin(fault_indicator_pin, false);
            return EXIT_SUCCESS;
        case READY_TO_RACE_ID:
            HAL_GPIO_WritePin(ready_to_race_indicator_pin, true);
            HAL_GPIO_WritePin(idle_indicator_pin, false);
            HAL_GPIO_WritePin(fault_indicator_pin, false);
            return EXIT_SUCCESS;
        case DISCHARGE_ID:
            HAL_GPIO_WritePin(ready_to_race_indicator_pin, false);
            HAL_GPIO_WritePin(idle_indicator_pin, false);
            HAL_GPIO_WritePin(fault_indicator_pin, false);
            return EXIT_SUCCESS;
        case FAULT_ID:
            HAL_GPIO_WritePin(ready_to_race_indicator_pin, false);
            HAL_GPIO_WritePin(idle_indicator_pin, false);
            HAL_GPIO_WritePin(fault_indicator_pin, true);
            return EXIT_SUCCESS;
        default:
            return EXIT_FAILURE;
    }
}

/**
 * @brief Force a transition into FAULT state.
 *
 * Attempts to open all contactors. Individual failures are recorded but do NOT
 * abort – a welded or stuck contactor must never prevent the system from
 * reaching the safe FAULT state.
 *
 * @param current_state  Pointer to the active FSM state (updated unconditionally).
 * @return EXIT_FAILURE if any contactor could not be opened, EXIT_SUCCESS otherwise.
 */
static bool FSM_EnterFault(FSM_State_t* current_state) {
    bool result = EXIT_SUCCESS;
    if (Contactor_SetState(&AIR_N,    OPEN) == EXIT_FAILURE) { result = EXIT_FAILURE; }
    if (Contactor_SetState(&PRECHARGE, OPEN) == EXIT_FAILURE) { result = EXIT_FAILURE; }
    if (Contactor_SetState(&AIR_P,    OPEN) == EXIT_FAILURE) { result = EXIT_FAILURE; }
    if (Contactor_SetState(&DISCHARGE, OPEN) == EXIT_FAILURE) { result = EXIT_FAILURE; }
    *current_state = FAULT_STATE;
    _update_timestamp(current_state);
    return result;
}

/**
 * @brief Transition the battery pack FSM to a new state
 *
 * This function performs the necessary contactor state changes to transition
 * from the current state to the new state. It should include safety checks and
 * timing requirements for transitions, such as ensuring precharge time is met.
 *
 * @param current_state Pointer to the current FSM state
 * @param new_state Pointer to the desired new FSM state
 * @return true if transition successful, false if transition failed or invalid
 */
bool FSM_Transition(FSM_State_t* current_state, const FSM_State_t* new_state) {
    if (!current_state || !new_state) {
        FSM_SetLED(&FAULT_STATE);
        return EXIT_FAILURE;
    }

    /* Check if transition is valid (this is a simplified example, real implementation should have more comprehensive checks) */
    if (_compare_contactor_states(current_state, new_state)) {
        FSM_SetLED(&new_state);
        return EXIT_SUCCESS; // Already in desired state
    }

    /* Perform necessary contactor state changes here with safety checks and timing requirements */
    switch (current_state->state_id) {
        case SAFE_ID:
            if (new_state->state_id == PRECHARGE_ID) {
                // Transition from SAFE to PRECHARGE
                if (Contactor_SetState(&AIR_P, OPEN) == EXIT_FAILURE) {
                    return EXIT_FAILURE; // Failed to open AIR_P contactor
                }
                if (Contactor_SetState(&DISCHARGE, OPEN) == EXIT_FAILURE) {
                    return EXIT_FAILURE; // Failed to open discharge contactor
                }
                if (Contactor_SetState(&AIR_N, CLOSED) == EXIT_FAILURE) {
                    return EXIT_FAILURE; // Failed to close AIR_N contactor
                }
                if (Contactor_SetState(&PRECHARGE, CLOSED) == EXIT_FAILURE) {
                    return EXIT_FAILURE; // Failed to close precharge contactor
                }
                
                // Update current state
                *current_state = *new_state;
                _update_timestamp(current_state);
                FSM_SetLED(&new_state);
                return EXIT_SUCCESS;
            } else if (new_state->state_id == FAULT_ID) {
                /* SAFE → FAULT: use shared helper so a stuck contactor cannot
                 * block entry into the safe FAULT state. */
                return FSM_EnterFault(current_state);
            } else {
                return EXIT_FAILURE; // Invalid transition from SAFE to requested state
            }
        case PRECHARGE_ID:
            if (new_state->state_id == READY_TO_RACE_ID) {
                // Transition from PRECHARGE to READY_TO_RACE
                if (Contactor_SetState(&PRECHARGE, OPEN) == EXIT_FAILURE) {
                    return EXIT_FAILURE; // Failed to open precharge contactor
                }
                if (Contactor_SetState(&AIR_N, CLOSED) == EXIT_FAILURE) {
                    return EXIT_FAILURE; // Failed to close AIR_N contactor
                }
                if (Contactor_SetState(&AIR_P, CLOSED) == EXIT_FAILURE) {
                    return EXIT_FAILURE; // Failed to close AIR_P contactor
                }
                if (Contactor_SetState(&DISCHARGE, OPEN) == EXIT_FAILURE) {
                    return EXIT_FAILURE; // Failed to open discharge contactor
                }
                
                // Update current state
                *current_state = *new_state;
                _update_timestamp(current_state);
                FSM_SetLED(&new_state);
                return EXIT_SUCCESS;
            } else if (new_state->state_id == DISCHARGE_ID) {
                // Transition from PRECHARGE to DISCHARGE (abort precharge, isolate pack, enable discharge)
                if (Contactor_SetState(&PRECHARGE, OPEN) == EXIT_FAILURE) {
                    return EXIT_FAILURE; // Failed to open precharge contactor
                }
                if (Contactor_SetState(&AIR_N, OPEN) == EXIT_FAILURE) {
                    return EXIT_FAILURE; // Failed to open AIR_N contactor
                }
                if (Contactor_SetState(&AIR_P, OPEN) == EXIT_FAILURE) {
                    return EXIT_FAILURE; // Failed to open AIR_P contactor
                }
                if (Contactor_SetState(&DISCHARGE, CLOSED) == EXIT_FAILURE) {
                    return EXIT_FAILURE; // Failed to close discharge contactor
                }
                
                // Update current state
                *current_state = *new_state;
                _update_timestamp(current_state);
                FSM_SetLED(&new_state);
                return EXIT_SUCCESS;
            } else if (new_state->state_id == FAULT_ID) {
                /* PRECHARGE → FAULT */
                return FSM_EnterFault(current_state);
            } else {
                return EXIT_FAILURE; // Invalid transition from PRECHARGE to requested state
            }
        case READY_TO_RACE_ID:
            if (new_state->state_id == DISCHARGE_ID) {
                // Transition from READY_TO_RACE to DISCHARGE (isolate pack, enable discharge)
                if (Contactor_SetState(&AIR_P, OPEN) == EXIT_FAILURE) {
                    return EXIT_FAILURE; // Failed to open AIR_P contactor
                }
                if (Contactor_SetState(&AIR_N, OPEN) == EXIT_FAILURE) {
                    return EXIT_FAILURE; // Failed to open AIR_N contactor
                }
                if (Contactor_SetState(&PRECHARGE, OPEN) == EXIT_FAILURE) {
                    return EXIT_FAILURE; // Failed to open precharge contactor
                }
                if (Contactor_SetState(&DISCHARGE, CLOSED) == EXIT_FAILURE) {
                    return EXIT_FAILURE; // Failed to close discharge contactor
                }

                // Update current state
                *current_state = *new_state;
                _update_timestamp(current_state);
                FSM_SetLED(&new_state);
                return EXIT_SUCCESS;
            } else if (new_state->state_id == SAFE_ID) {
                // Transition from RUNNING to SAFE (all contactors open)
                if (Contactor_SetState(&AIR_N, OPEN) == EXIT_FAILURE) {
                    return EXIT_FAILURE; // Failed to open AIR_N contactor
                }
                if (Contactor_SetState(&PRECHARGE, OPEN) == EXIT_FAILURE) {
                    return EXIT_FAILURE; // Failed to open precharge contactor
                }
                if (Contactor_SetState(&AIR_P, OPEN) == EXIT_FAILURE) {
                    return EXIT_FAILURE; // Failed to open AIR_P contactor
                }
                if (Contactor_SetState(&DISCHARGE, OPEN) == EXIT_FAILURE) {
                    return EXIT_FAILURE; // Failed to open discharge contactor
                }   

                // Update current state
                *current_state = *new_state;
                _update_timestamp(current_state);
                FSM_SetLED(&new_state);
                return EXIT_SUCCESS;
            } else if (new_state->state_id == FAULT_ID) {
                /* RUNNING → FAULT */
                return FSM_EnterFault(current_state);
            } else {
                return EXIT_FAILURE; // Invalid transition from RUNNING to requested state
            }
        case DISCHARGE_ID:
            if (new_state->state_id == SAFE_ID) {
                // Transition from DISCHARGE to SAFE (all contactors open)
                if (Contactor_SetState(&AIR_N, OPEN) == EXIT_FAILURE) {
                    return EXIT_FAILURE; // Failed to open AIR_N contactor
                }
                if (Contactor_SetState(&PRECHARGE, OPEN) == EXIT_FAILURE) {
                    return EXIT_FAILURE; // Failed to open precharge contactor
                }
                if (Contactor_SetState(&AIR_P, OPEN) == EXIT_FAILURE) {
                    return EXIT_FAILURE; // Failed to open AIR_P contactor
                }
                if (Contactor_SetState(&DISCHARGE, OPEN) == EXIT_FAILURE) {
                    return EXIT_FAILURE; // Failed to open discharge contactor
                }

                // Update current state                
                *current_state = *new_state;
                _update_timestamp(current_state);
                FSM_SetLED(&new_state);
                return EXIT_SUCCESS;
            } else if (new_state->state_id == FAULT_ID) {
                /* DISCHARGE → FAULT */
                return FSM_EnterFault(current_state);
            } else {
                return EXIT_FAILURE; // Invalid transition from DISCHARGE to requested state
            }
        case FAULT_ID:
            return EXIT_FAILURE; // No valid transitions from FAULT state
        default:
            exit(EXIT_FAILURE); // Invalid current state
    }
}
