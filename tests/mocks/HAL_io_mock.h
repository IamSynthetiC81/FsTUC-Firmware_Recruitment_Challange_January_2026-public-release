#ifndef HAL_IO_MOCK_H
#define HAL_IO_MOCK_H

#include "HAL_io.h"
#include "contactor.h"
#include <stdbool.h>
#include <stdint.h>

#include "CircuitSim.h"

/**
 * @brief Resets all fake GPIO pins to LOW and clears all registered physical links.
 * Best called in the Unity setUp() function.
 */
void Mock_HAL_GPIO_Reset(void);

/**
 * @brief Registers a "Virtual Wire" between two pins.
 * When the drive_contactor is closed, the sense_pin will automatically update.
 */
void Mock_HAL_LinkPins(SimContactor_t *link);

/**
 * @brief Manually sets a pin level from the "outside".
 * Useful for simulating hardware faults, button presses, or stuck contacts.
 */
void Mock_HAL_SetPinPhysically(GPIO_Pin_t pin, bool level);

/**
 * @brief Checks what pull-up/down configuration was last set on a pin.
 */
GPIO_Pull_t Mock_HAL_GetPullConfig(GPIO_Pin_t pin);

// Declarations of the secret functions
void internal_set_pin_level(GPIO_Port_t port, GPIO_PinNumber_t pin, bool level);

void internal_force_pin_level(GPIO_Port_t port, GPIO_PinNumber_t pin, bool level);

void internal_set_pin_pull(GPIO_Port_t port, GPIO_PinNumber_t pin, GPIO_Pull_t pull);

bool internal_get_pin_level(GPIO_Port_t port, GPIO_PinNumber_t pin);

GPIO_Pull_t internal_get_pin_pull(GPIO_Port_t port, GPIO_PinNumber_t pin);

void internal_reset_hal_state(void);

void internal_link_pins(GPIO_Pin_t drive_pin, SimContactor_t *contactor);

#endif /* HAL_IO_MOCK_H */