#ifndef FSTUC_FIRMWARE_RECRUITMENT_CIRCUITSIM_H
#define FSTUC_FIRMWARE_RECRUITMENT_CIRCUITSIM_H

#include <math.h>
#include <stdbool.h>
#include <contactor.h>

#define CIRCUITSIM_DEFAULT_TRAVEL_TIME_MS 100.0
#define CIRCUITSIM_DEFAULT_DEBOUNCE_TIME_MS 50.0

typedef struct {
    Contactor_t* contactor;
    bool welded_closed; // If true, this contactor is physically stuck CLOSED
    bool is_moving; // True if currently transitioning between states (for simulating mechanical delay)
    float travel_time_ms; // Time it takes to transition between OPEN and CLOSED
    double effective_resistance; // Effective resistance of the contactor based on current state (open/closed) and any welding
    ContactorState_t physical_state; // Physical contact state after travel delay
    ContactorState_t desired_state; // Target state for current transition
}SimContactor_t;



void time_step(float dt_s);

void CircuitSim_Init(Contactor_t *air_p, Contactor_t *air_n, Contactor_t *precharge, Contactor_t *discharge);

/**
 * @brief Reset the circuit simulation to initial state.
 * Clears all contactors, voltages, and currents.
 * Should be called between test cases.
 */
void CircuitSim_Cleanup(void);

double get_battery_voltage(void);
double get_vehicle_voltage(void);
double get_battery_current(void);

void CircuitSim_SetContactor(Contactor_t *contactor, ContactorState_t state);
void CircuitSim_WeldContactorClosed(Contactor_t *contactor, bool welded_closed);
void CircuitSim_SetLoadResistance(double resistance);
void CircuitSim_SetLoadCapacitance(double capacitance);
void CircuitSim_SetPrechargeResistance(double resistance);
void CircuitSim_SetDischargeResistance(double resistance);
void CircuitSim_SetContactorOpenResistance(double resistance);
void CircuitSim_SetContactorClosedResistance(double resistance);

/**
 * @brief Update all sense pins to match circuit simulation physical state.
 */
void CircuitSim_UpdateSensePins(void);

/**
 * @brief Update physics simulation based on HAL GPIO pin changes.
 * Maps GPIO drive pin changes to contactor state updates.
 *
 * @param port GPIO port number
 * @param pin GPIO pin number
 * @param level New pin level (true = HIGH, false = LOW)
 */
void CircuitSim_UpdateFromGPIO(unsigned int port, unsigned int pin, bool level);

/**
 * @brief Get the sensed state of a contactor (what the sense pin reads).
 * Returns the physical state after mechanical transition delay.
 *
 * @param contactor Pointer to the contactor
 * @return GPIO level that the sense pin should read (true = HIGH, false = LOW)
 */
bool CircuitSim_GetSensedLevel(Contactor_t* contactor);

/**
 * @brief Get the physical contact state (OPEN/CLOSED) after travel delay.
 *
 * @param contactor Pointer to the contactor
 * @return Physical state of the contactor
 */
ContactorState_t CircuitSim_GetPhysicalState(Contactor_t* contactor);

/**
 * @brief Check if a contactor is welded closed.
 *
 * @param contactor Pointer to the contactor
 * @return true if the contactor is welded closed, false otherwise
 */
bool CircuitSim_IsContactorWelded(Contactor_t* contactor);

#endif //FSTUC_FIRMWARE_RECRUITMENT_CIRCUITSIM_H