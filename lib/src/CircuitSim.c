#include "CircuitSim.h"

#include <stdio.h>
#include <stdlib.h>

#define THROW_STR(msg) \
    do { \
    fprintf(stderr, "Error %s:%d (%s): %s\n", \
    __FILE__, __LINE__, __func__, msg); \
    exit(EXIT_FAILURE); \
} while(0)


#include "HAL_io.h"
#include "HAL_io_mock.h"
static const float BATTERY_STATIC_VOLTAGE = 3.6*144;

static float V_BATTERY;
static float V_VEHICLE;
static float BATT_CURRENT;

SimContactor_t CircuitContactors[4];

double RESISTANCE_CONTACTOR_CLOSED = 0.01; // Ohms, very low when closed
double RESISTANCE_CONTACTOR_OPEN = 1e8; // Ohms, very high
double RESISTANCE_BATTERY_INTERNAL = 0.336; // Ohms, internal resistance of battery
double RESISTANCE_PRECHARGE = 3*750;
double RESISTANCE_DISCHARGE = 3*750;
double RESISTANCE_LOAD = 1e9;
double CAPACITANCE_LOAD = 0.0005;

double RA;
double RB;

double last_state_change_time_s = 0.0;
double V_VEHICLE_at_transition = 0.0;  // Initial voltage when transition started
double last_RA = 0.0;  // Track RA changes to detect topology changes

double parallel_resistance(const  double r1, const double r2) {
    if (r1 <= 0.0 || r2 <= 0.0) return 0.0;
    return (r1 * r2) / (r1 + r2);
}

double calc_v_inf() {
    const double CONTACTOR_RESISTANCE_AIR_P = CircuitContactors[0].effective_resistance;
    const double CONTACTOR_RESISTANCE_AIR_N = CircuitContactors[1].effective_resistance;
    const double CONTACTOR_RESISTANCE_PRECHARGE = CircuitContactors[2].effective_resistance;
    const double CONTACTOR_RESISTANCE_DISCHARGE = CircuitContactors[3].effective_resistance;

    /* Discharge path goes through the discharge contactor plus the configured discharge resistor. */
    RB = parallel_resistance(RESISTANCE_LOAD,
                             CONTACTOR_RESISTANCE_DISCHARGE + RESISTANCE_DISCHARGE);
    RA = RESISTANCE_BATTERY_INTERNAL + CONTACTOR_RESISTANCE_AIR_N + parallel_resistance(CONTACTOR_RESISTANCE_AIR_P, CONTACTOR_RESISTANCE_PRECHARGE + RESISTANCE_PRECHARGE);
    const double ratio = RB / (RA + RB);

    const double v_inf = BATTERY_STATIC_VOLTAGE * ratio;

    return v_inf;
}

double calc_tau() {
    const double tau = (RA * RB) / (RA + RB) * CAPACITANCE_LOAD;
    return tau;
}

double calc_v(const float dt_s) {
    const double v_inf = calc_v_inf();
    const double tau = calc_tau();

    // Use elapsed time since last transition
    // V(t) = V_inf + (V_initial - V_inf) * exp(-t/tau)
    // Use V_VEHICLE_at_transition as the initial voltage (captured when transition started)
    const double elapsed_time = last_state_change_time_s + dt_s;

    // Protect against divide by zero or negative tau
    if (tau <= 0.0) {
        THROW_STR("Invalid time constant tau calculated, check resistances and capacitance");
    }

    const double vc = v_inf + (V_VEHICLE_at_transition - v_inf) * exp(-elapsed_time / tau);
    return vc;
}

double calc_i() {
    if (RA > 1e20) {
        return 0.0;  // Essentially no current when circuit is open
    }
    return (BATTERY_STATIC_VOLTAGE - V_VEHICLE) / RA;
}

double calc_vb() {
    return BATTERY_STATIC_VOLTAGE - BATT_CURRENT * RESISTANCE_BATTERY_INTERNAL;
}

void calc(const float dt_s) {
    /* Compute circuit parameters first so RA/RB reflect any contactor changes. */
    const double v_inf = calc_v_inf();
    const double tau = calc_tau();

    const bool topology_changed = (RA != last_RA);
    if (topology_changed) {
        /* Capture the voltage at the instant of topology change for correct transient math. */
        V_VEHICLE_at_transition = V_VEHICLE;
        last_state_change_time_s = 0.0;
        last_RA = RA;
    }

    const double elapsed_time = last_state_change_time_s + dt_s;
    last_state_change_time_s += dt_s;

    const double new_V_VEHICLE = v_inf + (V_VEHICLE_at_transition - v_inf) * exp(-elapsed_time / tau);

    if (!isnan(new_V_VEHICLE)) {
        V_VEHICLE = new_V_VEHICLE;
    }

    const double new_BATT_CURRENT = calc_i();
    if (!isnan(new_BATT_CURRENT)) {
        BATT_CURRENT = new_BATT_CURRENT;
    }

    const double new_V_BATTERY = calc_vb();
    if (!isnan(new_V_BATTERY)) {
        V_BATTERY = new_V_BATTERY;
    }
}

void bounce_contact(const Contactor_t *contactor) {
    (void)contactor; // Suppress unused parameter warning
    //TODO : Implement contact bounce logic if needed, for now we assume ideal contactor behavior without bounce
}

void advance_travel_time(const float dt_s) {
    for (int i = 0; i < 4; i++) {
        if (CircuitContactors[i].is_moving) {
            CircuitContactors[i].travel_time_ms += dt_s * 1000.0;

            // Transition completes after travel_time_ms (debounce is handled within travel time, not after)
            if (CircuitContactors[i].travel_time_ms >= CIRCUITSIM_DEFAULT_TRAVEL_TIME_MS) {
                // Physical state transition occurs - use desired_state
                ContactorState_t new_state = CircuitContactors[i].desired_state;

                CircuitContactors[i].contactor->current_state = new_state;
                CircuitContactors[i].physical_state = new_state;

                // Update effective resistance based on NEW state
                CircuitContactors[i].effective_resistance = (new_state == OPEN) ? RESISTANCE_CONTACTOR_OPEN : RESISTANCE_CONTACTOR_CLOSED;

                bounce_contact(CircuitContactors[i].contactor);

                // Contactor ready for next command after travel time completes
                CircuitContactors[i].travel_time_ms = 0;
                CircuitContactors[i].is_moving = false;
                // Time reset is now handled in calc() when it detects RA changes
            }
        }
    }
}
void time_step(const float dt_s) {
    advance_travel_time(dt_s);
    calc(dt_s);
}

/**
 * @brief Initialize physics with specific battery voltage
 * @param air_p
 * @param air_n
 * @param precharge
 * @param discharge
 */
void CircuitSim_Init(Contactor_t *air_p, Contactor_t *air_n, Contactor_t *precharge, Contactor_t *discharge) {
    /* Reset every contactor to OPEN regardless of stale current_state. */
    if (air_p)      air_p->current_state      = OPEN;
    if (air_n)      air_n->current_state      = OPEN;
    if (precharge)  precharge->current_state  = OPEN;
    if (discharge)  discharge->current_state  = OPEN;

    const SimContactor_t air_p_sim    = { .contactor = air_p,    .welded_closed = false, .is_moving = false, .travel_time_ms = 0, .effective_resistance = RESISTANCE_CONTACTOR_OPEN, .physical_state = OPEN, .desired_state = OPEN };
    const SimContactor_t air_n_sim    = { .contactor = air_n,    .welded_closed = false, .is_moving = false, .travel_time_ms = 0, .effective_resistance = RESISTANCE_CONTACTOR_OPEN, .physical_state = OPEN, .desired_state = OPEN };
    const SimContactor_t precharge_sim = { .contactor = precharge, .welded_closed = false, .is_moving = false, .travel_time_ms = 0, .effective_resistance = RESISTANCE_CONTACTOR_OPEN, .physical_state = OPEN, .desired_state = OPEN };
    const SimContactor_t discharge_sim = { .contactor = discharge, .welded_closed = false, .is_moving = false, .travel_time_ms = 0, .effective_resistance = RESISTANCE_CONTACTOR_OPEN, .physical_state = OPEN, .desired_state = OPEN };

    CircuitContactors[0] = air_p_sim;
    CircuitContactors[1] = air_n_sim;
    CircuitContactors[2] = precharge_sim;
    CircuitContactors[3] = discharge_sim;

    // Reset global circuit state
    V_BATTERY = BATTERY_STATIC_VOLTAGE;
    V_VEHICLE = 0.0;
    V_VEHICLE_at_transition = 0.0;  // Initial voltage for exponential curve
    BATT_CURRENT = 0.0;
    last_state_change_time_s = 0.0;
    last_RA = 0.0;

    // Initial calculation
    calc(0.0);


}

/**
 * @brief Reset the circuit simulation to initial state.
 */
void CircuitSim_Cleanup(void) {
    // Reset all contactors
    for (int i = 0; i < 4; i++) {
        CircuitContactors[i].is_moving = false;
        CircuitContactors[i].travel_time_ms = 0.0;
        CircuitContactors[i].welded_closed = false;
        /* Reset physical / desired state so that the next setUp's
         * BatteryPack_Init → HAL_Delay_ms → CircuitSim_UpdateSensePins
         * sees all contactors OPEN rather than a stale closed state. */
        CircuitContactors[i].physical_state = OPEN;
        CircuitContactors[i].desired_state = OPEN;
        /* Sync effective resistance with the reset physical state. */
        CircuitContactors[i].effective_resistance = RESISTANCE_CONTACTOR_OPEN;
    }

    // Reset circuit state
    V_BATTERY = BATTERY_STATIC_VOLTAGE;
    V_VEHICLE = 0.0;
    V_VEHICLE_at_transition = 0.0;
    BATT_CURRENT = 0.0;
    last_state_change_time_s = 0.0;
    last_RA = 0.0;

    /* Set circuit to default values */
    RESISTANCE_CONTACTOR_CLOSED = 0.01; // Ohms, very low when closed
    RESISTANCE_CONTACTOR_OPEN = 1e8; // Ohms, very high
    RESISTANCE_BATTERY_INTERNAL = 0.336; // Ohms, internal resistance of battery
    RESISTANCE_PRECHARGE = 3*750;
    RESISTANCE_DISCHARGE = 3*750;
    RESISTANCE_LOAD = 1e9;
    CAPACITANCE_LOAD = 0.0005;
}

void CircuitSim_SetContactor(Contactor_t *contactor, const ContactorState_t state) {
    int index = -1;
    for (size_t i = 0; i < 4; i++) {
        if (CircuitContactors[i].contactor == contactor) {
            index = (int)i;
            break;
        }
    }

    if (index == -1) {
        fprintf(stderr, "Contactor not found in simulation\n");
        exit(-1);
    }

    // Check if contactor is welded closed
    if (CircuitContactors[index].welded_closed) {
        CircuitContactors[index].contactor->current_state = CLOSED;
        return;
    }

    if (CircuitContactors[index].contactor->current_state != state) {
        CircuitContactors[index].is_moving = true;
        CircuitContactors[index].travel_time_ms = 0.0;
        CircuitContactors[index].desired_state = state;
    }
}

void CircuitSim_WeldContactorClosed(Contactor_t *contactor, const bool welded_closed) {
    int index = -1;
    for (size_t i = 0; i < 4; i++) {
        if (CircuitContactors[i].contactor == contactor) {
            index = (int)i;
            break;
        }
    }

    if (index == -1) {
        THROW_STR("Contactor not found in simulation");
    }

    CircuitContactors[index].welded_closed = welded_closed;
    if (welded_closed) {
        // Stuck in CLOSED state
        CircuitContactors[index].contactor->current_state = CLOSED;
        CircuitContactors[index].physical_state = CLOSED;
        CircuitContactors[index].effective_resistance = RESISTANCE_CONTACTOR_CLOSED;
        CircuitContactors[index].is_moving = false;
        CircuitContactors[index].travel_time_ms = 0.0;
    }
}

void CircuitSim_SetLoadResistance(const double resistance) {
    RESISTANCE_LOAD = resistance;
    last_state_change_time_s = 0.0;
}

void CircuitSim_SetLoadCapacitance(const double capacitance) {
    CAPACITANCE_LOAD = capacitance;
    last_state_change_time_s = 0.0;
}

void CircuitSim_SetPrechargeResistance(const double resistance) {
    RESISTANCE_PRECHARGE = resistance;
    last_state_change_time_s = 0.0;
}

void CircuitSim_SetDischargeResistance(const double resistance) {
    RESISTANCE_DISCHARGE = resistance;
    last_state_change_time_s = 0.0;
}

void CircuitSim_SetContactorOpenResistance(const double resistance) {
    RESISTANCE_CONTACTOR_OPEN = resistance;
    last_state_change_time_s = 0.0;
}

void CircuitSim_SetContactorClosedResistance(const double resistance) {
    RESISTANCE_CONTACTOR_CLOSED = resistance;
    last_state_change_time_s = 0.0;
}

/**
 * @brief Update physics simulation based on HAL GPIO pin changes.
 * Called from HAL_GPIO_WritePin when contactor drive pins change.
 *
 * @param port GPIO port number
 * @param pin GPIO pin number
 * @param level New pin level (true = HIGH, false = LOW)
 */
void CircuitSim_UpdateFromGPIO(unsigned int port, unsigned int pin, bool level) {
    // Find matching contactor by drive pin
    for (int i = 0; i < 4; i++) {
        Contactor_t* c = CircuitContactors[i].contactor;
        if (!c) continue;

        // Check if this GPIO matches a contactor drive pin
        if (c->GPIO_Drive_pin.port == port && c->GPIO_Drive_pin.pinNumber == pin) {
            // Determine desired state based on drive logic
            bool wants_closed = (c->drive_logic == ACTIVE_HIGH) ? level : !level;
            ContactorState_t desired_state = wants_closed ? CLOSED : OPEN;

            // Update contactor state in physics
            CircuitSim_SetContactor(c, desired_state);
            break;
        }
    }
}

/**
 * @brief Get the sensed state of a contactor (what the sense pin reads).
 * Returns the physical state based on sense logic (ACTIVE_HIGH vs ACTIVE_LOW).
 *
 * @param contactor Pointer to the contactor
 * @return GPIO level that the sense pin should read (true = HIGH, false = LOW)
 */
bool CircuitSim_GetSensedLevel(Contactor_t* contactor) {
    if (!contactor) return false;

    for (int i = 0; i < 4; i++) {
        if (CircuitContactors[i].contactor == contactor) {
            ContactorState_t physical_state = CircuitContactors[i].physical_state;

            // If welded closed, always report CLOSED
            if (CircuitContactors[i].welded_closed) {
                physical_state = CLOSED;
            }

            // Determine electrical sense state based on normal state mapping
            ContactorState_t sense_electrical_state =
                (physical_state == contactor->normalState)
                    ? contactor->sense_normal_state
                    : (contactor->sense_normal_state == OPEN ? CLOSED : OPEN);

            bool sense_level = (contactor->sense_logic == ACTIVE_HIGH)
                                   ? (sense_electrical_state == CLOSED)
                                   : (sense_electrical_state == OPEN);

            return sense_level;
        }
    }

    return false;
}

ContactorState_t CircuitSim_GetPhysicalState(Contactor_t* contactor) {
    if (!contactor) return OPEN;

    for (int i = 0; i < 4; i++) {
        if (CircuitContactors[i].contactor == contactor) {
            ContactorState_t physical_state = CircuitContactors[i].physical_state;
            if (CircuitContactors[i].welded_closed) {
                physical_state = CLOSED;
            }
            return physical_state;
        }
    }

    return OPEN;
}

bool CircuitSim_IsContactorWelded(Contactor_t* contactor) {
    if (!contactor) return false;

    for (int i = 0; i < 4; i++) {
        if (CircuitContactors[i].contactor == contactor) {
            return CircuitContactors[i].welded_closed;
        }
    }

    return false;
}

/**
 * @brief Get the current battery voltage.
 * @return Battery voltage in volts
 */
double get_battery_voltage(void) {
    return V_BATTERY;
}

/**
 * @brief Get the current vehicle/load voltage.
 * @return Vehicle voltage in volts
 */
double get_vehicle_voltage(void) {
    return V_VEHICLE;
}

/**
 * @brief Get the current battery discharge current.
 * @return Battery current in amperes
 */
double get_battery_current(void) {
    return BATT_CURRENT;
}

/**
 * @brief Update all sense pins to reflect current physical contactor states.
 * Called during time_step to synchronize GPIO pins with circuit simulation.
 */
void CircuitSim_UpdateSensePins(void) {
    for (int i = 0; i < 4; i++) {
        if (CircuitContactors[i].contactor) {
            const bool sense_level = CircuitSim_GetSensedLevel(CircuitContactors[i].contactor);
            const GPIO_Pin_t sense_pin = CircuitContactors[i].contactor->GPIO_Sense_pin;

            // Update the GPIO sense pin to match physical state
            internal_set_pin_level(sense_pin.port, sense_pin.pinNumber, sense_level);
        }
    }
}

