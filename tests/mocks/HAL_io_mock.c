#include "HAL_io.h"
#include "HAL_io_mock.h"
#include <string.h>

#include "contactor.h"

/* --- Backdoor API --- */
void Mock_HAL_LinkPins(SimContactor_t *link) {
    if (!link || !link->contactor) return;
    // Pass pointer to the global/static SimContactor_t variable
    internal_link_pins(link->contactor->GPIO_Drive_pin, link);
}

void Mock_HAL_GPIO_Reset(void) {
    internal_reset_hal_state();
    CircuitSim_Cleanup();
}

void Mock_HAL_SetPinPhysically(GPIO_Pin_t pin, bool level) {
    internal_force_pin_level(pin.port, pin.pinNumber, level);
}

GPIO_Pull_t Mock_HAL_GetPullConfig(GPIO_Pin_t pin) {
    return internal_get_pin_pull(pin.port, pin.pinNumber);
}
