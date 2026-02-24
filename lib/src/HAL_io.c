#include "HAL_io.h"

#include <time.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>

#include <assert.h>
#include <stdio.h>

#include "CircuitSim.h"
#include "HAL_io_mock.h"

#ifdef UNIT_TESTING
extern void time_step(float dt_s);
#endif

/* ============================================================================
 * Clock State Variables
 * ========================================================================== */
static bool g_clock_initialized = false;
static uint64_t g_clock_start_ns = 0U;
static uint64_t g_clock_sim_ns = 0U;

/* ============================================================================
 * GPIO State Variables
 * ========================================================================== */
typedef struct {
    bool _is_initialized;
    GPIO_Mode_t _mode;
    GPIO_Pull_t _pull;
    bool _level;
    bool _forced_level; // If true, the level is forced to the given value (e.g. for simulating stuck pins or overrides)
    SimContactor_t* contactor;
} SimulatedPin_t;

static SimulatedPin_t simulated_pins[GPIO_PORT_COUNT][GPIO_PINS_PER_PORT] = {0};

/* ============================================================================
 * Helper Functions
 * ========================================================================== */

bool _ALL_VALID(GPIO_Pin_t pin) {
    return (pin.port >= GPIO_PORT_A && pin.port <= GPIO_PORT_C &&
            pin.pinNumber >= GPIO_PIN_0 && pin.pinNumber < GPIO_PINS_PER_PORT &&
            pin.mode >= GPIO_MODE_INPUT && pin.mode <= GPIO_MODE_ANALOG &&
            pin.pull >= GPIO_PULL_NO && pin.pull <= GPIO_PULL_DOWN);
}

long long _get_current_time_us() {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (long long)(ts.tv_sec) * 1000000LL + (long long)(ts.tv_nsec) / 1000LL;
}

/* ============================================================================
 * Clock Functions Implementation
 * ========================================================================== */

// TODO : This is a duplicate function. Refactor along with HAL_Clock_IsInitialized
static void HAL_Clock_EnsureInit(void) {
    if (!g_clock_initialized) {
        g_clock_sim_ns = 0U;
        g_clock_start_ns = 0U;
        g_clock_initialized = true;
    }
}

void HAL_Clock_Init(void) {
    g_clock_sim_ns = 0U;
    g_clock_start_ns = 0U;
    g_clock_initialized = true;
}

bool HAL_Clock_IsInitialized(void) {
    return g_clock_initialized;
}

/* ============================================================================
 * HAL Delay and Tick Functions
 * ========================================================================== */

void HAL_internalTransitionHandler(void);

void HAL_Delay_ms(uint32_t milliseconds) {
    HAL_Delay_us(milliseconds * 1000U);
}

void HAL_Delay_us(uint32_t microseconds) {
    HAL_Clock_EnsureInit();

    /* Advance HAL clock */
    g_clock_sim_ns += (uint64_t)microseconds * 1000ULL;

    #ifdef UNIT_TESTING
        const double elapsed_s = microseconds / 1000000.0;
        if (elapsed_s > 0.0) {
            time_step((float)elapsed_s);
        }
        // Update sense pins to reflect physical state from CircuitSim
        CircuitSim_UpdateSensePins();
    #endif

    /* Advance contactors travel time */
    for (int port = 0; port < GPIO_PORT_COUNT; port++) {
        for (int pin = 0; pin < GPIO_PINS_PER_PORT; pin++) {
            SimulatedPin_t* spin = &simulated_pins[port][pin];

            if (!spin->_is_initialized) continue;

            if (spin->contactor != NULL && spin->contactor->is_moving) {
                spin->contactor->travel_time_ms += (double)microseconds / 1000.0;
            }
        }
    }

    /* Update contactor sense pins if travel time completed */
    HAL_internalTransitionHandler();
}

uint64_t HAL_GetTick(void) {
    HAL_Clock_EnsureInit();
    return (g_clock_sim_ns * CLOCK_CPU_HZ) / 1000000000ULL;
}

double HAL_GetRuntimeMs(void) {
    HAL_Clock_EnsureInit();
    return (double)g_clock_sim_ns / 1000000.0;
}

/* ============================================================================
 * GPIO Functions Implementation
 * ========================================================================== */


void HAL_internalLinkHandler(SimulatedPin_t* spin, const bool drive_value) {
    if (!spin || !spin->contactor || !spin->contactor->contactor) {
        return;  // No contactor linked to this pin
    }

    // Check if contactor is welded closed - if so, it cannot move
    #ifdef UNIT_TESTING
    bool is_welded = CircuitSim_IsContactorWelded(spin->contactor->contactor);
    #else
    bool is_welded = spin->contactor->welded_closed;
    #endif

    if (is_welded) {
        /* Welded contactors are physically CLOSED regardless of the drive command.
         * Compute the aux contact state that corresponds to main=CLOSED:
         * - If main(CLOSED) == normalState → aux is in its normal state
         * - If main(CLOSED) != normalState → aux inverts from its normal state
         * This lets Contactor_GetState detect the drive/sense mismatch when the
         * contactor is commanded OPEN but physically stuck CLOSED. */
        GPIO_Port_t sense_port = spin->contactor->contactor->GPIO_Sense_pin.port;
        GPIO_PinNumber_t sense_pin = spin->contactor->contactor->GPIO_Sense_pin.pinNumber;

        const bool main_is_normal = (CLOSED == spin->contactor->contactor->normalState);
        ContactorState_t aux_state;
        if (main_is_normal) {
            aux_state = spin->contactor->contactor->sense_normal_state;
        } else {
            aux_state = (spin->contactor->contactor->sense_normal_state == OPEN) ? CLOSED : OPEN;
        }

        bool sense_level = (spin->contactor->contactor->sense_logic == ACTIVE_HIGH)
            ? (aux_state == CLOSED)
            : (aux_state == OPEN);

        internal_set_pin_level(sense_port, sense_pin, sense_level);
        spin->contactor->is_moving = false;
        return;
    }

    // Determine what the new main contact state would be
    ContactorState_t new_main_state = spin->contactor->contactor->drive_logic == ACTIVE_HIGH
        ? (drive_value ? CLOSED : OPEN)
        : (drive_value ? OPEN : CLOSED);

    // Only start a transition if the state is actually changing
    if (new_main_state != spin->contactor->contactor->current_state && !spin->contactor->is_moving) {
        spin->contactor->is_moving = true;
        spin->contactor->travel_time_ms = 0.0;
    }
}

void HAL_internalTransitionHandler() {
    for (int port = 0; port < GPIO_PORT_COUNT; port++) {
        for (int pin = 0; pin < GPIO_PINS_PER_PORT; pin++) {
            SimulatedPin_t* spin = &simulated_pins[port][pin];
            if (!spin->_is_initialized || !spin->contactor || !spin->contactor->contactor) {
                continue;
            }

            if (spin->contactor->travel_time_ms >= CIRCUITSIM_DEFAULT_TRAVEL_TIME_MS && spin->contactor->is_moving) {
                spin->contactor->travel_time_ms = 0;

                // Get current drive pin level
                const bool drive_level = internal_get_pin_level(
                    spin->contactor->contactor->GPIO_Drive_pin.port,
                    spin->contactor->contactor->GPIO_Drive_pin.pinNumber
                );

                // Check if contactor is welded closed (query CircuitSim for authoritative state)
                const bool is_welded = CircuitSim_IsContactorWelded(spin->contactor->contactor);

                // Determine main contact state from drive level
                const ContactorState_t main_state = spin->contactor->contactor->drive_logic == ACTIVE_HIGH
                    ? (drive_level ? CLOSED : OPEN)
                    : (drive_level ? OPEN : CLOSED);

                // Determine aux state: if main is normal, aux is in its normal state; else inverted
                bool main_is_normal = (main_state == spin->contactor->contactor->normalState);
                ContactorState_t aux_state;
                if (main_is_normal) {
                    aux_state = spin->contactor->contactor->sense_normal_state;
                } else {
                    aux_state = (spin->contactor->contactor->sense_normal_state == OPEN) ? CLOSED : OPEN;
                }

                // Convert aux state to electrical level
                bool sense_level;
                if (is_welded) {
                    // if welded, if main is in normal state, aux is in normal state; if main is not normal, aux is in inverted state
                    const bool inPhase = (spin->contactor->contactor->normalState == spin->contactor->contactor->sense_normal_state);
                    if (inPhase) {
                        sense_level = spin->contactor->contactor->sense_logic == ACTIVE_HIGH
                            ? (spin->contactor->contactor->normalState == CLOSED)
                            : (spin->contactor->contactor->normalState == OPEN);
                    } else {
                        sense_level = spin->contactor->contactor->sense_logic == ACTIVE_HIGH
                            ? (spin->contactor->contactor->normalState == OPEN)
                            : (spin->contactor->contactor->normalState == CLOSED);
                    }
                } else {
                    sense_level = spin->contactor->contactor->sense_logic == ACTIVE_HIGH
                    ? (aux_state == CLOSED)
                    : (aux_state == OPEN);
                }

                // Mark contactor as done moving
                spin->contactor->is_moving = false;

                const GPIO_Port_t sense_port = spin->contactor->contactor->GPIO_Sense_pin.port;
                const GPIO_PinNumber_t sense_pin_num = spin->contactor->contactor->GPIO_Sense_pin.pinNumber;

                internal_set_pin_level(sense_port, sense_pin_num, sense_level);
            }
        }
    }
}

bool HAL_GPIO_Init(GPIO_Pin_t pin) {
    // if all pins are valid, simulate success
    simulated_pins[pin.port][pin.pinNumber]._is_initialized = true;
    simulated_pins[pin.port][pin.pinNumber]._mode = pin.mode;
    simulated_pins[pin.port][pin.pinNumber]._pull = pin.pull;
    // simulated_pins[pin.port][pin.pinNumber].contactor = NULL;

    return _ALL_VALID(pin);
}

bool HAL_GPIO_WritePin(const GPIO_Pin_t pin, const bool value) {
    SimulatedPin_t* spin = &simulated_pins[pin.port][pin.pinNumber];

    if (!spin->_is_initialized) {
        return false; // Pin not initialized
    }
    if (spin->_mode == GPIO_MODE_ANALOG) {
        return false; // Cannot write digital value to analog pin
    }

    // Update the drive pin level FIRST
    spin->_level = value;

    // Notify CircuitSim of GPIO change
    #ifdef UNIT_TESTING
        CircuitSim_UpdateFromGPIO(pin.port, pin.pinNumber, value);
    #endif

    // Then handle contactor transition logic
    HAL_internalLinkHandler(spin, value);

    return _ALL_VALID(pin);
}

bool HAL_GPIO_ReadPin(GPIO_Pin_t pin) {
    if (!_ALL_VALID(pin)) {
        return false; // Invalid parameters
    }

    if (simulated_pins[pin.port][pin.pinNumber]._forced_level) {
        return simulated_pins[pin.port][pin.pinNumber]._level;
    }

    /* Check contactor runtime */
    HAL_internalTransitionHandler();

    return simulated_pins[pin.port][pin.pinNumber]._level;
}

/* ============================================================================
 * Internal/Secret Backdoor Functions (For Mock/Recruiter use only)
 * ========================================================================== */

void internal_set_pin_level(GPIO_Port_t port, GPIO_PinNumber_t pin, bool level) {
    assert(port < GPIO_PORT_COUNT && pin < GPIO_PINS_PER_PORT);
    simulated_pins[port][pin]._level = level;
    // Do NOT set _forced_level here - this is for contactor simulation updates
}

void internal_force_pin_level(GPIO_Port_t port, GPIO_PinNumber_t pin, bool level) {
    assert(port < GPIO_PORT_COUNT && pin < GPIO_PINS_PER_PORT);
    simulated_pins[port][pin]._level = level;
    simulated_pins[port][pin]._forced_level = true;  // Lock this pin to the forced value
}

bool internal_get_pin_level(GPIO_Port_t port, GPIO_PinNumber_t pin) {
    assert(port < GPIO_PORT_COUNT && pin < GPIO_PINS_PER_PORT);
    return simulated_pins[port][pin]._level;
}

void internal_reset_hal_state(void) {
    memset(simulated_pins, 0, sizeof(simulated_pins));
    g_clock_initialized = false;
    g_clock_start_ns = 0U;
    g_clock_sim_ns = 0U;
}

void internal_set_pin_pull(GPIO_Port_t port, GPIO_PinNumber_t pin, GPIO_Pull_t pull) {
    assert(port < GPIO_PORT_COUNT && pin < GPIO_PINS_PER_PORT);
    simulated_pins[port][pin]._pull = pull;
}

GPIO_Pull_t internal_get_pin_pull(GPIO_Port_t port, GPIO_PinNumber_t pin) {
    assert(port < GPIO_PORT_COUNT && pin < GPIO_PINS_PER_PORT);
    return simulated_pins[port][pin]._pull;
}

void internal_link_pins(GPIO_Pin_t drive_pin, SimContactor_t* contactor) {
    if (!contactor) return;

    GPIO_PinNumber_t pin_number = drive_pin.pinNumber;
    GPIO_Port_t port = drive_pin.port;

    if (port >= GPIO_PORT_COUNT || pin_number >= GPIO_PINS_PER_PORT) {
        return;  // Invalid pin
    }

    simulated_pins[port][pin_number]._forced_level = false;
    simulated_pins[port][pin_number].contactor = contactor;  // Store pointer to global/static memory

    // Initialize sense pin to correct state based on current drive pin level
    bool current_drive_level = simulated_pins[port][pin_number]._level;
    ContactorState_t current_main_state = contactor->contactor->drive_logic == ACTIVE_HIGH
        ? (current_drive_level ? CLOSED : OPEN)
        : (current_drive_level ? OPEN : CLOSED);

    // Determine aux contact state
    bool main_is_normal = (current_main_state == contactor->contactor->normalState);

    ContactorState_t aux_state;
    if (main_is_normal) {
        // Main is de-energized (in normal state) → aux is in its normal state
        aux_state = contactor->contactor->sense_normal_state;
    } else {
        // Main is energized (not normal) → aux inverts from its normal state
        aux_state = (contactor->contactor->sense_normal_state == OPEN) ? CLOSED : OPEN;
    }

    // Convert to electrical level
    bool sense_electrical = contactor->contactor->sense_logic == ACTIVE_HIGH
        ? (aux_state == CLOSED)
        : (aux_state == OPEN);

    // Set sense pin initial state
    GPIO_Port_t sense_port = contactor->contactor->GPIO_Sense_pin.port;
    GPIO_PinNumber_t sense_pin = contactor->contactor->GPIO_Sense_pin.pinNumber;
    internal_set_pin_level(sense_port, sense_pin, sense_electrical);
}
