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

/* ============================================================================
 * Transition Timer Management
 * ========================================================================== */
static uint32_t s_precharge_start_time = 0;
static uint32_t s_discharge_start_time = 0;

static uint32_t _GetTimeMs(void) {
    uint64_t ticks = HAL_GetTick();
    return (uint32_t)((ticks * 1000ULL) / CLOCK_CPU_HZ);
}

/**
 * @brief Reset precharge timer for new precharge cycle
 */
static void _Precharge_Reset(void) {
    s_precharge_start_time = 0;
}

/**
 * @brief Reset discharge timer for new discharge cycle
 */
static void _Discharge_Reset(void) {
    s_discharge_start_time = 0;
}

/**
 * @brief Check precharge completion status
 * @return 1 if precharge complete, -1 if timeout/failed, 0 if still in progress
 */
int Precharge(void){
    if (current_state.state_id != PRECHARGE_ID) {
        Error_Set(&BatteryPackErrorRegister, FAULT_BPACK_INVALID_TRANSITION);
        return TRANSITION_FAILED; // Not in precharge state
    }

    const uint32_t elapsed_time = _GetTimeMs() - current_state.timestamp_ms;


    // get battery voltage and vehicle voltage from ADC
    const uint16_t battery_voltage_adc = ADC_Read(&g_adc, 1);
    const uint16_t vehicle_voltage_adc = ADC_Read(&g_adc, 2);

    static const uint16_t BATTERY_MIN_VOLTAGE_ADC = (uint16_t)((2.5*144 * 4095.0) / 600.0); // Example minimum voltage to consider valid (10V)
    {
        /* Verify voltage is within bounds */
        if (battery_voltage_adc < BATTERY_MIN_VOLTAGE_ADC) {
            _Precharge_Reset(); // Reset for next cycle
            Error_Set(&BatteryPackErrorRegister, FAULT_BPACK_UNDERVOLTAGE);

            fprintf(stderr, "Precharge failed: Battery voltage too low (ADC: %u)\n", battery_voltage_adc);

            return TRANSITION_FAILED; // Invalid battery voltage
        }
        if (battery_voltage_adc > (uint16_t)((630.0 * 4095.0) / 600.0)) { // Example maximum voltage to consider valid (600V)
            _Precharge_Reset(); // Reset for next cycle
            Error_Set(&BatteryPackErrorRegister, FAULT_BPACK_OVERVOLTAGE);

            fprintf(stderr, "Precharge failed: Battery voltage too high (ADC: %u)\n", battery_voltage_adc);

            return TRANSITION_FAILED; // Invalid battery voltage
        }
    }

    const uint16_t voltage_threshold_adc = (uint16_t)(battery_voltage_adc * PRECHARGE_VOLTAGE_RATIO);

    static const uint32_t PRECHARGE_MIN_TIME_MS = 100; // Minimum time to ensure contactor has time to close
    

    if (elapsed_time > PRECHARGE_TIMEOUT_MS) {
        _Precharge_Reset(); // Reset for next cycle
        Error_Set(&BatteryPackErrorRegister, FAULT_BPACK_PRECHARGE_FAILURE);
        return TRANSITION_FAILED;
    } else if (vehicle_voltage_adc >= voltage_threshold_adc && elapsed_time >= PRECHARGE_MIN_TIME_MS) {
        _Precharge_Reset(); // Reset for next precharge cycle
        return TRANSITION_COMPLETE;
    }
    return TRANSITION_BUSY; // Still precharging
}

/**
 * @brief Check discharge completion status
 * For normal shutdowns, discharge completes quickly.
 * @return 1 if discharge complete, -1 if timeout/failed, 0 if still in progress  
 */
int Discharge(void){
    const uint32_t current_time = _GetTimeMs();

    static uint16_t vehicle_voltage_adc = 0;
    vehicle_voltage_adc = ADC_Read(&g_adc, 2);
    bool safe_voltage = vehicle_voltage_adc <= DISCHARGE_VOLTAGE_THRESHOLD_ADC;

    uint32_t elapsed_time = current_time - current_state.timestamp_ms;

    // Minimum discharge time to ensure contactor actually operates
    static const uint32_t DISCHARGE_MIN_TIME_MS = 100;

    if (elapsed_time + 1U >= DISCHARGE_TIMEOUT_MS) {
        _Discharge_Reset(); // Reset for next cycle
        Error_Set(&BatteryPackErrorRegister, FAULT_BPACK_DISCHARGE_FAILURE);
        return TRANSITION_FAILED;
    } else if (safe_voltage && elapsed_time >= DISCHARGE_MIN_TIME_MS) {
        _Discharge_Reset(); // Reset for next discharge cycle
        return TRANSITION_COMPLETE; // Discharge "complete" for test purposes
    }

    return TRANSITION_BUSY; // Still discharging
}

bool BatteryPack_Init(void) {
    /* Clear any residual fault state from a previous run so that the error
     * register does not carry over between test cases or power cycles. */
    Error_ClearRegister(&BatteryPackErrorRegister);

    /* Initialize Pins  */
    drive_command_pin = (GPIO_Pin_t){
        .port = GPIO_PORT_C,
        .pinNumber = GPIO_PIN_0,
        .mode = GPIO_MODE_INPUT,
        .pull = GPIO_PULL_DOWN
    };
    fault_indicator_pin = (GPIO_Pin_t){
        .port = GPIO_PORT_C,
        .pinNumber = GPIO_PIN_1,
        .mode = GPIO_MODE_OUTPUT,
        .pull = GPIO_PULL_DOWN
    };
    ready_to_race_indicator_pin = (GPIO_Pin_t){
        .port = GPIO_PORT_C,
        .pinNumber = GPIO_PIN_2,
        .mode = GPIO_MODE_OUTPUT,
        .pull = GPIO_PULL_DOWN
    };
    idle_indicator_pin = (GPIO_Pin_t){
        .port = GPIO_PORT_C,
        .pinNumber = GPIO_PIN_3,
        .mode = GPIO_MODE_OUTPUT,
        .pull = GPIO_PULL_DOWN
    };

    /* Initialize contactors pins */
    AIR_P_drive_pin = (GPIO_Pin_t){
        .port = GPIO_PORT_A,
        .pinNumber = GPIO_PIN_0,
        .mode = GPIO_MODE_OUTPUT,
        .pull = GPIO_PULL_DOWN
    };
    AIR_N_drive_pin = (GPIO_Pin_t){
        .port = GPIO_PORT_A,
        .pinNumber = GPIO_PIN_1,
        .mode = GPIO_MODE_OUTPUT,
        .pull = GPIO_PULL_DOWN
    };
    PRECHARGE_drive_pin = (GPIO_Pin_t){
        .port = GPIO_PORT_A,
        .pinNumber = GPIO_PIN_2,
        .mode = GPIO_MODE_OUTPUT,
        .pull = GPIO_PULL_DOWN      
    };
    DISCHARGE_drive_pin = (GPIO_Pin_t){
        .port = GPIO_PORT_A,
        .pinNumber = GPIO_PIN_3,
        .mode = GPIO_MODE_OUTPUT,
        .pull = GPIO_PULL_DOWN
    };

    AIR_P_AUX_pin = (GPIO_Pin_t){
        .port = GPIO_PORT_B,
        .pinNumber = GPIO_PIN_0,
        .mode = GPIO_MODE_INPUT,
        .pull = GPIO_PULL_DOWN
    };
    AIR_N_AUX_pin = (GPIO_Pin_t){
        .port = GPIO_PORT_B,
        .pinNumber = GPIO_PIN_1,
        .mode = GPIO_MODE_INPUT,
        .pull = GPIO_PULL_DOWN
    };
    PRECHARGE_AUX_pin = (GPIO_Pin_t){
        .port = GPIO_PORT_B,
        .pinNumber = GPIO_PIN_2,
        .mode = GPIO_MODE_INPUT,
        .pull = GPIO_PULL_DOWN
    };
    DISCHARGE_AUX_pin = (GPIO_Pin_t){
        .port = GPIO_PORT_B,
        .pinNumber = GPIO_PIN_3,
        .mode = GPIO_MODE_INPUT,
        .pull = GPIO_PULL_DOWN
    };
    
    /* Initialize contactors to open state */
    
    AIR_P = (Contactor_t){
        .GPIO_Drive_pin = AIR_P_drive_pin,
        .drive_logic = ACTIVE_HIGH,
        .main_normal_state = OPEN,
        .GPIO_AUX_pin = AIR_P_AUX_pin,
        .aux_logic = ACTIVE_HIGH,
        .aux_normal_state = CLOSED,
        .current_state = OPEN,
        .persistance_data = {
            .cycle_count = 0,
            .EEPROM_cycle_count_address = 0x0100,
            .EEPROM_track_cycle_count = true,
            .accumulated_runtime = 0,
            .EEPROM_accumulated_runtime_address = 0x0200,
            .EEPROM_track_runtime = true,
            .runtime = 0
        },
        .fault_register = {
            .peripheralName = "AIR_P",
            .errorReg = 0
        }
    };

    AIR_N = (Contactor_t){
        .GPIO_Drive_pin = AIR_N_drive_pin,
        .drive_logic = ACTIVE_HIGH,
        .main_normal_state = OPEN,
        .GPIO_AUX_pin = AIR_N_AUX_pin,
        .aux_logic = ACTIVE_HIGH,
        .aux_normal_state = CLOSED,
        .current_state = OPEN,
        .persistance_data = {
            .cycle_count = 0,
            .EEPROM_cycle_count_address = 0x0104,
            .EEPROM_track_cycle_count = true,
            .accumulated_runtime = 0,
            .EEPROM_accumulated_runtime_address = 0x0204,
            .EEPROM_track_runtime = true,
            .runtime = 0
        },
        .fault_register = {
            .peripheralName = "AIR_N",
            .errorReg = 0
        }
    };

    PRECHARGE = (Contactor_t){
        .GPIO_Drive_pin = PRECHARGE_drive_pin,
        .drive_logic = ACTIVE_HIGH,
        .main_normal_state = OPEN,
        .GPIO_AUX_pin = PRECHARGE_AUX_pin,
        .aux_logic = ACTIVE_HIGH,
        .aux_normal_state = CLOSED,
        .current_state = OPEN,
        .persistance_data = {
            .cycle_count = 0,
            .EEPROM_cycle_count_address = 0x0108,
            .EEPROM_track_cycle_count = true,
            .accumulated_runtime = 0,
            .EEPROM_accumulated_runtime_address = 0x0208,
            .EEPROM_track_runtime = true,
            .runtime = 0
        },
        .fault_register = {
            .peripheralName = "PRECHARGE",
            .errorReg = 0
        }
    };

    DISCHARGE = (Contactor_t){
        .GPIO_Drive_pin = DISCHARGE_drive_pin,
        .drive_logic = ACTIVE_HIGH,
        .main_normal_state = OPEN,
        .GPIO_AUX_pin = DISCHARGE_AUX_pin,
        .aux_logic = ACTIVE_HIGH,
        .aux_normal_state = CLOSED,
        .current_state = OPEN,
        .persistance_data = {
            .cycle_count = 0,
            .EEPROM_cycle_count_address = 0x010C,  /* After protected region */
            .EEPROM_track_cycle_count = true,
            .accumulated_runtime = 0,
            .EEPROM_accumulated_runtime_address = 0x020C,
            .EEPROM_track_runtime = true,
            .runtime = 0
        },
        .fault_register = {
            .peripheralName = "DISCHARGE",
            .errorReg = 0
        }
    };


    if (Contactor_Init(&AIR_P) == EXIT_FAILURE) Error_Set(&BatteryPackErrorRegister, FAULT_BPACK_AIR_P_FAILURE);
    if (Contactor_Init(&AIR_N) == EXIT_FAILURE) Error_Set(&BatteryPackErrorRegister, FAULT_BPACK_AIR_N_FAILURE);
    if (Contactor_Init(&PRECHARGE) == EXIT_FAILURE) Error_Set(&BatteryPackErrorRegister, FAULT_BPACK_PRECHARGE_FAILURE);
    if (Contactor_Init(&DISCHARGE) == EXIT_FAILURE) Error_Set(&BatteryPackErrorRegister, FAULT_BPACK_DISCHARGE_FAILURE);

    if (Error_Match(&BatteryPackErrorRegister, FAULT_BPACK_HARD_MASK)) {
        fprintf(stderr, "BatteryPack_Init: Contactor initialization failed with error code 0x%02X\n", BatteryPackErrorRegister.errorReg);
        return EXIT_FAILURE;
    }

    /* Initialize ADC for monitoring pack voltage, vehicle voltage, and current */
    ADC_Config_t adc_cfg = {
        .mode = ADC_MODE_POLLING,
        .resolution = 0x02,  /* 12-bit resolution */
        .channelEnable = 0x07  /* Enable channels 1, 2, and 3 */
    };

    if (!ADC_Init(&g_adc, &adc_cfg)) {
        Error_Set(&BatteryPackErrorRegister, FAULT_BPACK_ADC_FAILURE_BIT);
        return EXIT_FAILURE;
    }

    /* Set the FSM to the initial SAFE state so that current_state.air_*_state
     * fields reflect OPEN contactors.  Without this, the zero-initialised
     * global has all contactor fields set to CLOSED (0), which causes the
     * first FSM_Transition(SAFE→SAFE) call to see a contactor-state mismatch
     * and fall into the invalid-transition else-branch, returning EXIT_FAILURE. */
    current_state = SAFE_STATE;

    return EXIT_SUCCESS;
}

/**
 *  GOTO Safe state and indicate fault condition. This should be called when any critical fault is detected, such as contactor failure, overvoltage, undervoltage, or overcurrent conditions.
 * 
 * Also, tries to identify the fault and set appropriate indicators (e.g., fault LED) to help with diagnostics.
 */
bool BatteryPack_FaultHandler(void) {
    if (!Error_Match(&BatteryPackErrorRegister, FAULT_BPACK_HARD_MASK)) {
        fprintf(stderr, "Fault Handler without error set - this should not happen\n");
    }

    switch (current_state.state_id) {
        case SAFE_ID:
            FSM_Transition(&current_state, &FAULT_STATE);

            // Set fault indicator LED
            HAL_GPIO_WritePin(fault_indicator_pin, true);
            HAL_GPIO_WritePin(ready_to_race_indicator_pin, false);
            HAL_GPIO_WritePin(idle_indicator_pin, false);


            break;
        case PRECHARGE_ID:
            FSM_Transition(&current_state, &DISCHARGE_STATE);
            break;
        case READY_TO_RACE_ID:
            FSM_Transition(&current_state, &DISCHARGE_STATE);
            break;
        case DISCHARGE_ID:
            do {
                if (Error_Match(&BatteryPackErrorRegister, FAULT_BPACK_HARD_MASK)) {
                    // special case
                }
            }while (Precharge() == TRANSITION_COMPLETE);
            break;
        case FAULT_ID:
            // Already in FAULT state - likely a critical fault that requires manual reset
            break;
        default:
            // Unknown state - treat as critical fault
            break;
    }
    while(1) {
        // Stay in fault state until reset
    }
}

bool BatteryPack_FSM(void) {
    const bool DRIVE_COMMAND = HAL_GPIO_ReadPin(drive_command_pin);

    switch(current_state.state_id) {
        case SAFE_ID: {
            if (DRIVE_COMMAND && !Error_Match(&BatteryPackErrorRegister, FAULT_BPACK_HARD_MASK)) {
                _Precharge_Reset();
                const bool transition_fault = FSM_Transition(&current_state, &PRECHARGE_STATE);
                if (transition_fault == EXIT_FAILURE) {
                    /* Contactor fault during transition to PRECHARGE */
                    Error_Set(&BatteryPackErrorRegister, FAULT_BPACK_TRANSITION_FAULT);
                    FSM_Transition(&current_state, &FAULT_STATE);
                    return EXIT_FAILURE;
                }
                    return EXIT_SUCCESS;
            } else {
                return FSM_Transition(&current_state, &SAFE_STATE);
            }
        }
        case PRECHARGE_ID: {
            /* Check precharge completion status */
            const int precharge_status = Precharge();
            if (precharge_status == TRANSITION_COMPLETE) {
                const bool transition_ok = FSM_Transition(&current_state, &READY_TO_RACE_STATE);
                if (transition_ok == EXIT_FAILURE) {
                    /* Contactor fault during transition to READY_TO_RACE */
                    Error_Set(&BatteryPackErrorRegister, FAULT_BPACK_TRANSITION_FAULT);
                    FSM_Transition(&current_state, &FAULT_STATE);
                }
                return EXIT_SUCCESS;
            } else if (precharge_status == TRANSITION_FAILED) {
                Error_Set(&BatteryPackErrorRegister, FAULT_BPACK_TRANSITION_FAULT);
                return FSM_Transition(&current_state, &FAULT_STATE);
            } else if (DRIVE_COMMAND && !Error_Match(&BatteryPackErrorRegister, FAULT_BPACK_HARD_MASK)) {
                /* Still precharging */
                return FSM_Transition(&current_state, &PRECHARGE_STATE);
            } else {
                /* Drive command lost during precharge - transition to discharge for safe shutdown */
                _Precharge_Reset();
                _Discharge_Reset();
                return FSM_Transition(&current_state, &DISCHARGE_STATE);
            }
        }
        case READY_TO_RACE_ID: {
            if (DRIVE_COMMAND && !Error_Match(&BatteryPackErrorRegister, FAULT_BPACK_HARD_MASK)) {
                return FSM_Transition(&current_state, &READY_TO_RACE_STATE);
            } else {
                /* Shutdown requested - transition to discharge state */
                _Discharge_Reset();
                const bool transition_ok = FSM_Transition(&current_state, &DISCHARGE_STATE);
                if (transition_ok == EXIT_FAILURE) {
                    /* Contactor fault detected during transition */
                    Error_Set(&BatteryPackErrorRegister, FAULT_BPACK_TRANSITION_FAULT);
                    FSM_Transition(&current_state, &FAULT_STATE);
                }
                return EXIT_SUCCESS;
            }
        }
        case DISCHARGE_ID: {
            /* Check discharge progress regardless of drive command */
            int discharge_status = Discharge();
            if (discharge_status == 1) {
                /* Discharge complete - go to SAFE */
                const bool transition_ok = FSM_Transition(&current_state, &SAFE_STATE);
                if (transition_ok == EXIT_FAILURE) {
                    /* Contactor fault during transition to SAFE */
                    Error_Set(&BatteryPackErrorRegister, FAULT_BPACK_TRANSITION_FAULT);
                    FSM_Transition(&current_state, &FAULT_STATE);
                }
                return EXIT_SUCCESS;
            } else if (discharge_status == -1) {
                /* Discharge failed/timeout - go to FAULT */
                Error_Set(&BatteryPackErrorRegister, FAULT_BPACK_TRANSITION_FAULT);
                return FSM_Transition(&current_state, &FAULT_STATE);
            } else {
                /* Still discharging - stay in DISCHARGE */
                return FSM_Transition(&current_state, &DISCHARGE_STATE);
            }
        }
        case FAULT_ID: {
            /* FAULT state is sticky - stay in FAULT no matter what */
            return FSM_Transition(&current_state, &FAULT_STATE);
        }
        default: {
            return FSM_Transition(&current_state, &FAULT_STATE);
        }
    }
}
