/********************
 * Battery Pack Control
 * 
 * This module manages the battery pack's contactors and monitors for faults.
 * It implements a simple FSM to control the contactor states based on ADC readings.
 * 
 * The ADC is set so that it takes the following measurements:
 * - Channel 1: Pack voltage (0-600V scaled to 12-bit range)
 * - Channel 2: Vehicle Voltage (0-600V scaled to 12-bit range)
 * - Channel 3: Pack current (±200A scaled to 12-bit range with offset)
 * 
 * Pin assignments for contactors:
 * - AIR_P Drive:       GPIOA Pin 0 (Active High)
 * - AIR_N Drive:       GPIOA Pin 1 (Active High)
 * - PRECHARGE Drive:   GPIOA Pin 2 (Active High)
 * - DISCHARGE Drive:   GPIOA Pin 3 (Active High)
 *
 * - AIR_P Sense:       GPIOB Pin 0 (Active High)
 * - AIR_N Sense:       GPIOB Pin 1 (Active High)
 * - PRECHARGE Sense:   GPIOB Pin 2 (Active High)
 * - DISCHARGE Sense:   GPIOB Pin 3 (Active High)
 *
 * Pin assignemts for drive signals and Indicator LEDs:
 * - PACK_START_COMMAND: GPIOC Pin 0 (Active High)
 * - FAULT_INDICATOR_LED: GPIOC Pin 1 (Active High)
 * - RUNNING_INDICATOR_LED: GPIOC Pin 2 (Active High)
 * - IDLE_INDICATOR_LED: GPIOC Pin 3 (Active High)
 * 
 * The FSM has the following states:
 * - SAFE: All contactors open, waiting for start command
 * - PRECHARGE: Precharge contactor closed, monitoring voltage rise
 * - RUNNING: Main contactors closed, normal operation
 * - DISCHARGE: Main contactors closed, discharge contactor closed for regenerative braking
 * - FAULT: All contactors open, fault detected
 * 
 * Transitions:
 * All transitions should be executed with appropriate timing and safety checks, such as ensuring precharge time is met
 * and monitoring for faults during transitions.
 * 
 * - Transitions towards FAULT or SAFE, must first go through DISCHARGE state to ensure safe opening of contactors, even in the case of a welded contactor fault. For example:
 *  - From RUNNING to SAFE: RUNNING -> DISCHARGE -> SAFE
 * 
 * 
 * EEPROM Memory Map:
 * 0x0000 - 0x00FF: Reserved
 * 0x0100 - 0x01FF: Contactor Cycle Counts (4 bytes per contactor)
 * 0x0200 - 0x02FF: Contactor Accumulated Runtime (4 bytes per contactor)
 * 0x0300 - 0x03FF: Reserved for future use
 * 
 * Fault Handling:
 * - If a fault is detected (e.g., precharge timeout, discharge timeout, invalid ADC readings), the system should transition to the FAULT state, open all contactors, and set the appropriate fault bits in the error register. 
 * - The FAULT state should require a manual reset (e.g., power cycle) to exit, ensuring that faults are not ignored and are properly addressed.
 * 
 * Safety Considerations:
 * - All contactor state changes should be verified through the sense pins to ensure that the physical state matches the commanded state. If a mismatch is detected, a fault should be set.
 *  
 * !!! THERE MUST BE NO AMBIGUITY IN THE FSM TRANSITIONS. Each state should have clear and deterministic transitions based on the inputs and conditions.
 *     FAULT state can be entered from any state if a fault condition is detected, and SAFE state can only be entered from DISCHARGE state after ensuring safe conditions.
 * 
 ********************/

#include "BatteryPack.h"
#include "FSM.h"
#include "CircuitSim.h"

/* ============================================================================
 * Global Variable Definitions
 * ========================================================================== */
ErrorRegister_t BatteryPackErrorRegister = {
    .peripheralName = "BatteryPack",
    .errorReg = 0
};

Contactor_t AIR_P;
Contactor_t AIR_N;
Contactor_t PRECHARGE;
Contactor_t DISCHARGE;

ADC_Handle_t g_adc;

GPIO_Pin_t drive_command_pin;
GPIO_Pin_t AIR_P_AUX_pin;
GPIO_Pin_t AIR_N_AUX_pin;
GPIO_Pin_t PRECHARGE_AUX_pin;
GPIO_Pin_t DISCHARGE_AUX_pin;

GPIO_Pin_t AIR_P_drive_pin;
GPIO_Pin_t AIR_N_drive_pin;
GPIO_Pin_t PRECHARGE_drive_pin;
GPIO_Pin_t DISCHARGE_drive_pin;

GPIO_Pin_t fault_indicator_pin;
GPIO_Pin_t ready_to_race_indicator_pin;
GPIO_Pin_t idle_indicator_pin;

FSM_State_t current_state;




bool BatteryPack_Init(void) {
   
}

/**
 *  GOTO Safe state and indicate fault condition. This should be called when any critical fault is detected, such as contactor failure, overvoltage, undervoltage, or overcurrent conditions.
 * 
 * Also, tries to identify the fault and set appropriate indicators (e.g., fault LED) to help with diagnostics.
 */
bool BatteryPack_FaultHandler(void) {
    
}

bool BatteryPack_FSM(void) {
    
}
