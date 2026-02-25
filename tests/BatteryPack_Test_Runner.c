#include "unity.h"
#include "BatteryPack.h"
#include "HardwareMoch.h"
#include "EEPROM.h"
#include "FSM.h"
#include "CircuitSim.h"

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#ifndef LOG_PATH
#define LOG_PATH "./tests/log/"
#endif

static inline bool hasHardFaults(void) {
    return Error_Match(&BatteryPackErrorRegister, FAULT_BPACK_HARD_MASK);
}

static inline bool hasAnyFaults(void) {
    return BatteryPackErrorRegister.errorReg != 0;
}

static FILE* s_log_file = NULL;
static double s_time_s = 0.0;

static SimContactor_t air_p_sim, air_n_sim, precharge_sim, discharge_sim;

static const char* _state_to_string(FSM_id_t state_id) {
    switch (state_id) {
        case SAFE_ID: return "SAFE";
        case PRECHARGE_ID: return "PRECHARGE";
        case RUNNING_ID: return "RUNNING";
        case DISCHARGE_ID: return "DISCHARGE";
        case FAULT_ID: return "FAULT";
        default: return "UNKNOWN";
    }
}

// static const char* normalize_log_path(const char* raw_path, char* buffer, size_t buffer_size) {
//     size_t length = strlen(raw_path);
//     size_t start = 0;
//     size_t end = length;

//     if (length > 0 && raw_path[0] == '"') {
//         start = 1;
//     }

//     if (end > start && raw_path[end - 1] == '"') {
//         end--;
//     }

//     if (end > start) {
//         size_t copy_len = end - start;

//         if (copy_len >= buffer_size) {
//             copy_len = buffer_size - 1;
//         }

//         memcpy(buffer, &raw_path[start], copy_len);
//         buffer[copy_len] = '\0';
//     } else if (buffer_size > 0U) {
//         buffer[0] = '\0';
//     }

//     return buffer;
// }

static void log_open(const char* filename) {
    char path[512];
    
    snprintf(path, sizeof(path), "%s%s", LOG_PATH, filename);
    
    /* Create directory if it doesn't exist */
    char mkdir_cmd[600];
    snprintf(mkdir_cmd, sizeof(mkdir_cmd), "mkdir -p \"%s\"", LOG_PATH);
    (void)system(mkdir_cmd);
    
    s_log_file = fopen(path, "w");
    TEST_ASSERT_NOT_NULL_MESSAGE(s_log_file, "Failed to open log file");
    fprintf(s_log_file, "time_s,battery_v,vehicle_v,pack_a,air_p,air_n,precharge,discharge,fsm_state,fault_detected\n");
}

static void log_step(void) {
    if (!s_log_file) {
        return;
    }

    fprintf(s_log_file, "%.6f,%.3f,%.3f,%.3f,%d,%d,%d,%d,%d,%s\n",
            s_time_s,
            get_battery_voltage(),
            get_vehicle_voltage(),
            get_battery_current(),
            AIR_P.current_state, AIR_N.current_state, PRECHARGE.current_state,
            DISCHARGE.current_state, hasHardFaults(), _state_to_string(current_state.state_id));
}

/**
 * @brief Wrapper around unified timing that maintains original sim_steps semantics.
 *
 * Replaces the old sim_steps function, now using consolidated timing internally.
 */
static void sim_steps(const double duration_s, const double dt_s) {
    const int steps = (int)(duration_s / dt_s);
    const double remainder = duration_s - (steps * dt_s);

    for (int i = 0; i < steps; i++) {
        /* Use unified timestep function */
        Test_Timestep_WithFSM_Ms((uint32_t)(dt_s * 1000.0));
        s_time_s += dt_s;
        log_step();
    }

    if (remainder > 0.0) {
        /* Final step */
        Test_Timestep_WithFSM_Ms((uint32_t)(remainder * 1000.0));
        s_time_s += remainder;
        log_step();
    }
}

static void drive_command(const bool enable) {
    Mock_HAL_SetPinPhysically(drive_command_pin, enable);
}

void ResetContactorsWithSim(void) {
    AIR_P = (Contactor_t){
        .GPIO_Drive_pin = {.port = GPIO_PORT_A, .pinNumber = GPIO_PIN_0, .pull = GPIO_PULL_DOWN},
        .drive_logic = ACTIVE_HIGH,
        .normalState = OPEN,
        .GPIO_Sense_pin = {.port = GPIO_PORT_B, .pinNumber = GPIO_PIN_0, .pull = GPIO_PULL_DOWN},
        .sense_logic = ACTIVE_HIGH,
        .sense_normal_state = CLOSED,
        .persistance_data = {
            .cycle_count = 0,
            .EEPROM_cycle_count_address = EEPROM_ADDRESS_CONTACTOR_CYCLE_COUNT + 0,
            .EEPROM_track_cycle_count = true,
            .accumulated_runtime = 0,
            .EEPROM_accumulated_runtime_address = EEPROM_ADDRESS_CONTACTOR_ACCURUNTIME + 0,
            .EEPROM_track_runtime = true
        },
        .fault_register = {
            .peripheralName = "AIR_P",
            .errorReg = 0
        }
    };

    AIR_N = (Contactor_t){
        .GPIO_Drive_pin = {.port = GPIO_PORT_A, .pinNumber = GPIO_PIN_1, .pull = GPIO_PULL_DOWN},
        .drive_logic = ACTIVE_HIGH,
        .normalState = OPEN,
        .GPIO_Sense_pin = {.port = GPIO_PORT_B, .pinNumber = GPIO_PIN_1, .pull = GPIO_PULL_DOWN},
        .sense_logic = ACTIVE_HIGH,
        .sense_normal_state = CLOSED,
        .persistance_data = {
            .cycle_count = 0,
            .EEPROM_cycle_count_address = EEPROM_ADDRESS_CONTACTOR_CYCLE_COUNT + 10,
            .EEPROM_track_cycle_count = true,
            .accumulated_runtime = 0,
            .EEPROM_accumulated_runtime_address = EEPROM_ADDRESS_CONTACTOR_ACCURUNTIME + 10,
            .EEPROM_track_runtime = true
        },
        .fault_register = {
            .peripheralName = "AIR_N",
            .errorReg = 0
        }
    };
    PRECHARGE = (Contactor_t){
        .GPIO_Drive_pin = {.port = GPIO_PORT_A, .pinNumber = GPIO_PIN_2, .pull = GPIO_PULL_DOWN},
        .drive_logic = ACTIVE_HIGH,
        .normalState = OPEN,
        .GPIO_Sense_pin = {.port = GPIO_PORT_B, .pinNumber = GPIO_PIN_2, .pull = GPIO_PULL_DOWN},
        .sense_logic = ACTIVE_HIGH,
        .sense_normal_state = CLOSED,
        .persistance_data = {
            .cycle_count = 0,
            .EEPROM_cycle_count_address = EEPROM_ADDRESS_CONTACTOR_CYCLE_COUNT + 20,
            .EEPROM_track_cycle_count = true,
            .accumulated_runtime = 0,
            .EEPROM_accumulated_runtime_address = EEPROM_ADDRESS_CONTACTOR_ACCURUNTIME + 20,
            .EEPROM_track_runtime = true
        },
        .fault_register = {
            .peripheralName = "PRECHARGE_CONTACTOR",
            .errorReg = 0
        }
    };
    DISCHARGE = (Contactor_t){
        .GPIO_Drive_pin = {.port = GPIO_PORT_A, .pinNumber = GPIO_PIN_3, .pull = GPIO_PULL_DOWN},
        .drive_logic = ACTIVE_HIGH,
        .normalState = OPEN,
        .GPIO_Sense_pin = {.port = GPIO_PORT_B, .pinNumber = GPIO_PIN_3, .pull = GPIO_PULL_DOWN},
        .sense_logic = ACTIVE_HIGH,
        .sense_normal_state = CLOSED,
        .persistance_data = {
            .cycle_count = 0,
            .EEPROM_cycle_count_address = EEPROM_ADDRESS_CONTACTOR_CYCLE_COUNT + 30,
            .EEPROM_track_cycle_count = true,
            .accumulated_runtime = 0,
            .EEPROM_accumulated_runtime_address = EEPROM_ADDRESS_CONTACTOR_ACCURUNTIME + 30,
            .EEPROM_track_runtime = true
        },
        .fault_register = {
            .peripheralName = "DISCHARGE_CONTACTOR",
            .errorReg = 0
        }
    };

    air_p_sim = (SimContactor_t){
        .contactor = &AIR_P,
        .physical_state = OPEN,
        .welded_closed = false,
        .is_moving = false,
        .travel_time_ms = 0,
        .effective_resistance = 1e9
    };
    air_n_sim = (SimContactor_t){
        .contactor = &AIR_N,
        .physical_state = OPEN,
        .welded_closed = false,
        .is_moving = false,
        .travel_time_ms = 0,
        .effective_resistance = 1e9
    };
    precharge_sim = (SimContactor_t){
        .contactor = &PRECHARGE,
        .physical_state = OPEN,
        .welded_closed = false,
        .is_moving = false,
        .travel_time_ms = 0,
        .effective_resistance = 1e9
    };
    discharge_sim = (SimContactor_t){
        .contactor = &DISCHARGE,
        .physical_state = OPEN,
        .welded_closed = false,
        .is_moving = false,
        .travel_time_ms = 0,
        .effective_resistance = 1e9
    };
}


// Global SimContactor_t structures to persist beyond setUp
static SimContactor_t air_p_sim;
static SimContactor_t air_n_sim;
static SimContactor_t precharge_sim;
static SimContactor_t discharge_sim;

void setUp(void) {
    Mock_HAL_GPIO_Reset();
    EEPROM_Init();
    EEPROM_Erase(0, EEPROM_TOTAL_SIZE_BYTES);

    drive_command(false);
    Error_ClearRegister(&AIR_P.fault_register);
    Error_ClearRegister(&AIR_N.fault_register);
    Error_ClearRegister(&PRECHARGE.fault_register);
    Error_ClearRegister(&DISCHARGE.fault_register);

    s_time_s = 0.0;

    HAL_Clock_Init();

    /* Initialize contactor structs with known configuration so that
     * Mock_HAL_LinkPins and BatteryPack_Init both see consistent data. */
    ResetContactorsWithSim();

    /* Initialize the circuit simulator BEFORE BatteryPack_Init so that
     * Contactor_Init → Contactor_SetState → HAL_Delay_ms →
     * CircuitSim_UpdateSensePins sees correct physical states (all OPEN).
     * CircuitSim_Init resets physical_state / desired_state to OPEN. */
    CircuitSim_Init(&AIR_P, &AIR_N, &PRECHARGE, &DISCHARGE);

    /* Link contactor drive pins to the HAL simulator BEFORE BatteryPack_Init
     * so that Contactor_Init → Contactor_SetState → sense-pin read works
     * correctly (the simulator sets the sense pin to the right level when
     * the drive pin changes). */
    Mock_HAL_LinkPins(&air_p_sim);
    Mock_HAL_LinkPins(&air_n_sim);
    Mock_HAL_LinkPins(&precharge_sim);
    Mock_HAL_LinkPins(&discharge_sim);

    /* BatteryPack_Init re-initialises the contactor structs, calls
     * Contactor_Init internally, clears BatteryPackErrorRegister, and
     * sets current_state = SAFE_STATE. */
    TEST_ASSERT_EQUAL(EXIT_SUCCESS, BatteryPack_Init());
}

void tearDown(void) {
    if (s_log_file) {
        fclose(s_log_file);
        s_log_file = NULL;
    }

    // Cleanup CircuitSim state
    CircuitSim_Cleanup();

    // Cleanup GPIO
    Mock_HAL_GPIO_Reset();
}

void test_BatteryPack_Init_Success(void) {
    /**
     * Verify that BatteryPack_Init correctly initializes.   
    */

    TEST_ASSERT_FALSE(hasAnyFaults());
}    

void test_BatteryPack_Initial_State(void) {
    /**
     * Verify that after initialization, the FSM is in the SAFE state and all contactors are OPEN. This confirms that the system starts in a known safe configuration.
     * Also verifies that the pack is configured with the expected pin assignments and logic, according to the design specifications.
     */

    {
        // Check FSM state
        TEST_ASSERT_EQUAL_UINT8(SAFE_ID, current_state.state_id);

        /* Verify that all contactors are in their expected initial state */
        TEST_ASSERT_EQUAL_UINT8(OPEN, AIR_P.current_state);
        TEST_ASSERT_EQUAL_UINT8(OPEN, AIR_N.current_state);
        TEST_ASSERT_EQUAL_UINT8(OPEN, PRECHARGE.current_state);
        TEST_ASSERT_EQUAL_UINT8(OPEN, DISCHARGE.current_state);
    }

    {
        /* Verify pin assignments and configurations for the peripherals */
        TEST_ASSERT_EQUAL_UINT8(GPIO_PORT_C, drive_command_pin.port);
        TEST_ASSERT_EQUAL_UINT8(GPIO_PIN_0, drive_command_pin.pinNumber);
        TEST_ASSERT_EQUAL_UINT8(GPIO_MODE_INPUT, drive_command_pin.mode);
        TEST_ASSERT_EQUAL_UINT8(GPIO_PULL_DOWN, drive_command_pin.pull);

        TEST_ASSERT_EQUAL_UINT8(GPIO_PORT_C, fault_indicator_pin.port);
        TEST_ASSERT_EQUAL_UINT8(GPIO_PIN_1, fault_indicator_pin.pinNumber);
        TEST_ASSERT_EQUAL_UINT8(GPIO_MODE_OUTPUT, fault_indicator_pin.mode);
        TEST_ASSERT_EQUAL_UINT8(GPIO_PULL_DOWN, fault_indicator_pin.pull);

        TEST_ASSERT_EQUAL_UINT8(GPIO_PORT_C, running_indicator_pin.port);
        TEST_ASSERT_EQUAL_UINT8(GPIO_PIN_2, running_indicator_pin.pinNumber);
        TEST_ASSERT_EQUAL_UINT8(GPIO_MODE_OUTPUT, running_indicator_pin.mode);
        TEST_ASSERT_EQUAL_UINT8(GPIO_PULL_DOWN, running_indicator_pin.pull);
    }

    {
        /* Verify pin assignments and configurations for each contactor */
        TEST_ASSERT_EQUAL_UINT8(GPIO_PORT_A, AIR_P_drive_pin.port);
        TEST_ASSERT_EQUAL_UINT8(GPIO_PIN_0, AIR_P_drive_pin.pinNumber);
        TEST_ASSERT_EQUAL_UINT8(GPIO_MODE_OUTPUT, AIR_P_drive_pin.mode);
        TEST_ASSERT_EQUAL_UINT8(GPIO_PULL_DOWN, AIR_P_drive_pin.pull);

        TEST_ASSERT_EQUAL_UINT8(GPIO_PORT_B, AIR_P_sense_pin.port);
        TEST_ASSERT_EQUAL_UINT8(GPIO_PIN_0, AIR_P_sense_pin.pinNumber);
        TEST_ASSERT_EQUAL_UINT8(GPIO_MODE_INPUT, AIR_P_sense_pin.mode);
        TEST_ASSERT_EQUAL_UINT8(GPIO_PULL_DOWN, AIR_P_sense_pin.pull);

        TEST_ASSERT_EQUAL_UINT8(GPIO_PORT_A, AIR_N_drive_pin.port);
        TEST_ASSERT_EQUAL_UINT8(GPIO_PIN_1, AIR_N_drive_pin.pinNumber);
        TEST_ASSERT_EQUAL_UINT8(GPIO_MODE_OUTPUT, AIR_N_drive_pin.mode);
        TEST_ASSERT_EQUAL_UINT8(GPIO_PULL_DOWN, AIR_N_drive_pin.pull);  

        TEST_ASSERT_EQUAL_UINT8(GPIO_PORT_B, AIR_N_sense_pin.port);
        TEST_ASSERT_EQUAL_UINT8(GPIO_PIN_1, AIR_N_sense_pin.pinNumber);
        TEST_ASSERT_EQUAL_UINT8(GPIO_MODE_INPUT, AIR_N_sense_pin.mode);
        TEST_ASSERT_EQUAL_UINT8(GPIO_PULL_DOWN, AIR_N_sense_pin.pull);

        TEST_ASSERT_EQUAL_UINT8(GPIO_PORT_A, PRECHARGE_drive_pin.port);
        TEST_ASSERT_EQUAL_UINT8(GPIO_PIN_2, PRECHARGE_drive_pin.pinNumber);
        TEST_ASSERT_EQUAL_UINT8(GPIO_MODE_OUTPUT, PRECHARGE_drive_pin.mode);
        TEST_ASSERT_EQUAL_UINT8(GPIO_PULL_DOWN, PRECHARGE_drive_pin.pull);

        TEST_ASSERT_EQUAL_UINT8(GPIO_PORT_B, PRECHARGE_sense_pin.port);
        TEST_ASSERT_EQUAL_UINT8(GPIO_PIN_2, PRECHARGE_sense_pin.pinNumber);
        TEST_ASSERT_EQUAL_UINT8(GPIO_MODE_INPUT, PRECHARGE_sense_pin.mode);
        TEST_ASSERT_EQUAL_UINT8(GPIO_PULL_DOWN, PRECHARGE_sense_pin.pull);      

        TEST_ASSERT_EQUAL_UINT8(GPIO_PORT_A, DISCHARGE_drive_pin.port);
        TEST_ASSERT_EQUAL_UINT8(GPIO_PIN_3, DISCHARGE_drive_pin.pinNumber);
        TEST_ASSERT_EQUAL_UINT8(GPIO_MODE_OUTPUT, DISCHARGE_drive_pin.mode);
        TEST_ASSERT_EQUAL_UINT8(GPIO_PULL_DOWN, DISCHARGE_drive_pin.pull);  

        TEST_ASSERT_EQUAL_UINT8(GPIO_PORT_B, DISCHARGE_sense_pin.port);
        TEST_ASSERT_EQUAL_UINT8(GPIO_PIN_3, DISCHARGE_sense_pin.pinNumber);
        TEST_ASSERT_EQUAL_UINT8(GPIO_MODE_INPUT, DISCHARGE_sense_pin.mode);
        TEST_ASSERT_EQUAL_UINT8(GPIO_PULL_DOWN, DISCHARGE_sense_pin.pull);
    }

    {
       /* Verify contactor logic is correctly initialized */
       TEST_ASSERT_EQUAL_UINT8(OPEN, AIR_P.current_state);
       TEST_ASSERT_EQUAL_UINT8(OPEN, AIR_N.current_state);
       TEST_ASSERT_EQUAL_UINT8(OPEN, PRECHARGE.current_state);
       TEST_ASSERT_EQUAL_UINT8(OPEN, DISCHARGE.current_state);
    }

    {
        /* Verify the memory map is followed */
        TEST_ASSERT_EQUAL_UINT16(0x0100, AIR_P.persistance_data.EEPROM_cycle_count_address);
        TEST_ASSERT_EQUAL_UINT16(0x0104, AIR_N.persistance_data.EEPROM_cycle_count_address);
        TEST_ASSERT_EQUAL_UINT16(0x0108, PRECHARGE.persistance_data.EEPROM_cycle_count_address);
        TEST_ASSERT_EQUAL_UINT16(0x010C, DISCHARGE.persistance_data.EEPROM_cycle_count_address);

        TEST_ASSERT_EQUAL_UINT16(0x0200, AIR_P.persistance_data.EEPROM_accumulated_runtime_address);
        TEST_ASSERT_EQUAL_UINT16(0x0204, AIR_N.persistance_data.EEPROM_accumulated_runtime_address);
        TEST_ASSERT_EQUAL_UINT16(0x0208, PRECHARGE.persistance_data.EEPROM_accumulated_runtime_address);
        TEST_ASSERT_EQUAL_UINT16(0x020C, DISCHARGE.persistance_data.EEPROM_accumulated_runtime_address);
    }

    {
        /* Verify track cycle count, runtime, and accumulated runtime are zero */
        TEST_ASSERT_EQUAL_UINT32(0, AIR_P.persistance_data.cycle_count);
        TEST_ASSERT_EQUAL_UINT32(0, AIR_N.persistance_data.cycle_count);
        TEST_ASSERT_EQUAL_UINT32(0, PRECHARGE.persistance_data.cycle_count);
        TEST_ASSERT_EQUAL_UINT32(0, DISCHARGE.persistance_data.cycle_count);

        TEST_ASSERT_EQUAL_UINT32(0, AIR_P.persistance_data.runtime);
        TEST_ASSERT_EQUAL_UINT32(0, AIR_N.persistance_data.runtime);
        TEST_ASSERT_EQUAL_UINT32(0, PRECHARGE.persistance_data.runtime);
        TEST_ASSERT_EQUAL_UINT32(0, DISCHARGE.persistance_data.runtime);

        TEST_ASSERT_EQUAL_UINT32(0, AIR_P.persistance_data.accumulated_runtime);
        TEST_ASSERT_EQUAL_UINT32(0, AIR_N.persistance_data.accumulated_runtime);
        TEST_ASSERT_EQUAL_UINT32(0, PRECHARGE.persistance_data.accumulated_runtime);
        TEST_ASSERT_EQUAL_UINT32(0, DISCHARGE.persistance_data.accumulated_runtime);
    }

    {
        /* Verify ADC configuration */
        TEST_ASSERT_EQUAL_UINT8(ADC_MODE_POLLING, g_adc.config.mode);
        TEST_ASSERT_EQUAL_UINT8(ADC_RESOLUTION_12BIT, g_adc.config.resolution);
        TEST_ASSERT_EQUAL_UINT8((1<<0) | (1<<1) | (1<<2), g_adc.config.channelEnable);
    }

    {
        /* Verify that no faults are present on initialization */
        TEST_ASSERT_FALSE_MESSAGE(hasAnyFaults(), "Expected no faults on initialization");
    }
        

}

void test_BatteryPack_Safe_To_Precharge_Transition(void) {
    /**
     * Verify that when a drive command is given, the FSM transitions from SAFE to PRECHARGE state, and the PRECHARGE contactor is commanded to close. This confirms that the system correctly initiates the precharge sequence in response to a drive request.
     * Also verifies that the drive command input is correctly read and processed by the FSM, and that the PRECHARGE contactor's drive pin is activated according to its active logic configuration.
     */

     {
        // Check initial state
        TEST_ASSERT_EQUAL_UINT8(SAFE_ID, current_state.state_id);
        TEST_ASSERT_EQUAL_UINT8(OPEN, PRECHARGE.current_state);
     }

     {
        /* Verify that drive command is initially LOW and FSM does not transition */
        TEST_ASSERT_FALSE(HAL_GPIO_ReadPin(drive_command_pin));
        TEST_ASSERT_EQUAL(EXIT_SUCCESS, BatteryPack_FSM());
        TEST_ASSERT_EQUAL_UINT8(SAFE_ID, current_state.state_id);
     }

     {
        /* Simulate drive command going HIGH and verify FSM transitions to PRECHARGE */
        drive_command(true);
        TEST_ASSERT_EQUAL(EXIT_SUCCESS, BatteryPack_FSM());
        TEST_ASSERT_EQUAL_UINT8(PRECHARGE_ID, current_state.state_id);
    }

    {
        /* Verify that PRECHARGE contactor is commanded to close according to its drive logic */
        const bool expected_drive_state = (PRECHARGE.drive_logic == ACTIVE_HIGH) ? true : false;
        TEST_ASSERT_EQUAL(expected_drive_state, HAL_GPIO_ReadPin(PRECHARGE.GPIO_Drive_pin));
    }

    {
        /* Verify that no faults are present during this transition */
        TEST_ASSERT_FALSE_MESSAGE(hasAnyFaults(), "Expected no faults during SAFE to PRECHARGE transition");
    }
}

void test_BatteryPack_Precharge_To_Running_Transition(void) {
    /**
     * Verify that after the PRECHARGE contactor is closed and the precharge timer has elapsed, the FSM transitions to the RUNNING state, and the AIR_P and AIR_N contactors are commanded to close. This confirms that the system correctly completes the precharge sequence and enters the running state in response to a drive request.
     * Also verifies that the precharge timer is correctly implemented and that the FSM only transitions to RUNNING after the required precharge time has elapsed, and that the AIR_P and AIR_N contactors are commanded to close according to their drive logic.
     */

    {
        // Simulate drive command going HIGH and verify FSM transitions to PRECHARGE
        drive_command(true);
        TEST_ASSERT_EQUAL(EXIT_SUCCESS, BatteryPack_FSM());
        TEST_ASSERT_EQUAL_UINT8(PRECHARGE_ID, current_state.state_id);
    }

    {
        /* Simulate precharge timer elapsing and verify FSM transitions to RUNNING */
        sim_steps(5.01, 0.001); // Simulate 5 seconds for precharge
        TEST_ASSERT_EQUAL_UINT8(RUNNING_ID, current_state.state_id);
    }

    {
        /* Verify that AIR_P and AIR_N contactors are commanded to close according to their drive logic */
        const bool expected_air_p_drive_state = (AIR_P.drive_logic == ACTIVE_HIGH) ? true : false;
        const bool expected_air_n_drive_state = (AIR_N.drive_logic == ACTIVE_HIGH) ? true : false;
        TEST_ASSERT_EQUAL(expected_air_p_drive_state, HAL_GPIO_ReadPin(AIR_P.GPIO_Drive_pin));
        TEST_ASSERT_EQUAL(expected_air_n_drive_state, HAL_GPIO_ReadPin(AIR_N.GPIO_Drive_pin));
    }

    {
        /* Verify that no faults are present during this transition */
        TEST_ASSERT_FALSE_MESSAGE(hasAnyFaults(), "Expected no faults during PRECHARGE to RUNNING transition");
    }

}

void test_BatteryPack_Running_To_Discharge_Transition(void) {

    /**
     * Verify that when the drive command is removed while in the RUNNING state, the FSM transitions to the DISCHARGE state, and the DISCHARGE contactor is commanded to close while AIR_P and AIR_N remain closed. This confirms that the system correctly initiates the discharge sequence in response to a drive-off event.
     * Also verifies that the drive command input is correctly read and processed by the FSM, and that the DISCHARGE contactor is commanded to close according to its drive logic while AIR_P and
     * 
     * AIR_N remain in their commanded state, ensuring a proper transition to the discharge state without prematurely opening the main contactors.
     */
    
        {
            // Simulate drive command going HIGH and verify FSM transitions to PRECHARGE
            drive_command(true);
            TEST_ASSERT_EQUAL(EXIT_SUCCESS, BatteryPack_FSM());
            TEST_ASSERT_EQUAL_UINT8(PRECHARGE_ID, current_state.state_id);
    
            // Simulate precharge timer elapsing and verify FSM transitions to RUNNING
            sim_steps(PRECHARGE_TIMEOUT_MS/1000.0 + 0.001, 0.001); // Simulate 5 seconds for precharge
            TEST_ASSERT_EQUAL_UINT8(RUNNING_ID, current_state.state_id);
        }

        {
            /* Simulate drive command going LOW and verify FSM transitions to DISCHARGE */
            drive_command(false);
            TEST_ASSERT_EQUAL(EXIT_SUCCESS, BatteryPack_FSM());
            TEST_ASSERT_EQUAL_UINT8(DISCHARGE_ID, current_state.state_id);
        }

        {
            /* In DISCHARGE state: DISCHARGE contactor is CLOSED, AIR_P and AIR_N are OPEN
             * (main contactors are opened first to isolate the pack, then DISCHARGE closes
             *  to allow capacitor bleed-down through the discharge resistor). */
            const bool expected_discharge_drive_state = (DISCHARGE.drive_logic == ACTIVE_HIGH) ? true : false;
            /* OPEN contactor → drive de-energised */
            const bool expected_air_p_drive_state = (AIR_P.drive_logic == ACTIVE_HIGH) ? false : true;
            const bool expected_air_n_drive_state = (AIR_N.drive_logic == ACTIVE_HIGH) ? false : true;

            TEST_ASSERT_EQUAL(expected_discharge_drive_state, HAL_GPIO_ReadPin(DISCHARGE.GPIO_Drive_pin));
            TEST_ASSERT_EQUAL(expected_air_p_drive_state, HAL_GPIO_ReadPin(AIR_P.GPIO_Drive_pin));
            TEST_ASSERT_EQUAL(expected_air_n_drive_state, HAL_GPIO_ReadPin(AIR_N.GPIO_Drive_pin));
        }

        {
            /* Verify that no faults are present during this transition */
            TEST_ASSERT_FALSE_MESSAGE(hasAnyFaults(), "Expected no faults during RUNNING to DISCHARGE transition");
        }
}

void test_BatteryPack_Normal_Cycle(void) {
    log_open("battery_pack_normal.csv");
    log_step();

    drive_command(false);
    sim_steps(1.0, 0.001);

    TEST_ASSERT_EQUAL(EXIT_SUCCESS, BatteryPack_FSM());
    TEST_ASSERT_EQUAL_UINT8(SAFE_ID, current_state.state_id);

    drive_command(true);
    TEST_ASSERT_EQUAL(EXIT_SUCCESS, BatteryPack_FSM());
    TEST_ASSERT_EQUAL_UINT8(PRECHARGE_ID, current_state.state_id);

    /* Simulate 7s drive-on window (precharge then running) */
    sim_steps(7.0, 0.001);

    TEST_ASSERT_EQUAL_UINT8_MESSAGE(
        RUNNING_ID, current_state.state_id, 
        "Expected to transition to RUNNING after precharge"
    );

    /* Request shutdown */
    drive_command(false);
    
    /* Simulate 7s drive-off window (discharge then safe) */
    sim_steps(7.0, 0.001);
    
    /* Verify FSM reached SAFE state */
    TEST_ASSERT_TRUE_MESSAGE(
        current_state.state_id == SAFE_ID
        , "Expected to be in SAFE state after shutdown"
    );
    
    /* Verify no faults were detected */
    TEST_ASSERT_FALSE_MESSAGE(
        hasAnyFaults()        , "Expected no faults during normal cycle"
    );
}

void test_BatteryPack_Shutdown_Uses_Discharge_Contactor(void) {
    /**
     * Verify that when shutting down from the RUNNING state, the FSM commands the DISCHARGE
     * contactor to close. AIR_P and AIR_N are opened as part of the DISCHARGE state transition
     * to isolate the battery pack while the load capacitor discharges through the discharge resistor.
     */

    /* Enable drive command so the FSM can advance through PRECHARGE → RUNNING */
    drive_command(true);

    double start_ms = HAL_GetRuntimeMs();
    do {
        sim_steps(0.01, 0.001);

        double elapsed_ms = HAL_GetRuntimeMs() - start_ms;
        TEST_ASSERT_TRUE_MESSAGE(elapsed_ms < (double)(PRECHARGE_TIMEOUT_MS + 1000U),
            "FSM did not transition to RUNNING within expected time"
        );
    } while (current_state.state_id < RUNNING_ID);

    const uint32_t discharge_cycle_count_before = DISCHARGE.persistance_data.cycle_count;

    /* Remove drive command to trigger shutdown → DISCHARGE */
    drive_command(false);

    TEST_ASSERT_EQUAL(EXIT_SUCCESS, BatteryPack_FSM());
    TEST_ASSERT_EQUAL_UINT8(DISCHARGE_ID, current_state.state_id);

    /* In DISCHARGE state: DISCHARGE is CLOSED, AIR_P and AIR_N are OPEN */
    const bool expected_discharge_drive_state = (DISCHARGE.drive_logic == ACTIVE_HIGH) ? true : false;
    const bool expected_air_p_drive_state     = (AIR_P.drive_logic    == ACTIVE_HIGH) ? false : true;
    const bool expected_air_n_drive_state     = (AIR_N.drive_logic    == ACTIVE_HIGH) ? false : true;

    TEST_ASSERT_EQUAL(expected_discharge_drive_state, HAL_GPIO_ReadPin(DISCHARGE.GPIO_Drive_pin));
    TEST_ASSERT_EQUAL(expected_air_p_drive_state,     HAL_GPIO_ReadPin(AIR_P.GPIO_Drive_pin));
    TEST_ASSERT_EQUAL(expected_air_n_drive_state,     HAL_GPIO_ReadPin(AIR_N.GPIO_Drive_pin));

    /* Verify that the discharge contactor's cycle count has incremented */
    TEST_ASSERT_EQUAL_UINT32_MESSAGE(
        discharge_cycle_count_before + 1, DISCHARGE.persistance_data.cycle_count,
        "Expected DISCHARGE contactor cycle count to increment during shutdown"
    );

    /* Verify no faults were detected */
    TEST_ASSERT_FALSE_MESSAGE(hasAnyFaults(), "Expected no faults during normal cycle");
}

    


/* ============================================================================
 * Fault Circuit Tests: Using CircuitSim to simulate faults
 * ========================================================================== */

void test_BatteryPack_Fault_01_AIR_P_Welded_After_First_Close(void) {
    /**
     * Verify that if AIR_P welds closed while in RUNNING, the FSM detects the fault
     * during the RUNNING→DISCHARGE transition (when it tries to open AIR_P) and
     * transitions to the FAULT state via DISCHARGE.
     */

    /* Advance through PRECHARGE → RUNNING */
    drive_command(true);
    do {
        sim_steps(0.01, 0.001);
        TEST_ASSERT_EQUAL(EXIT_SUCCESS, BatteryPack_FSM());
    } while (current_state.state_id < RUNNING_ID);

    /* Weld AIR_P closed - it can no longer open */
    CircuitSim_WeldContactorClosed(&AIR_P, true);

    /* Remove drive command: FSM attempts RUNNING→DISCHARGE, calls
     * Contactor_SetState(&AIR_P, OPEN).  The sense pin stays LOW (OPEN
     * reported while actually welded CLOSED) → DRIVE_SENSE_MISMATCH.
     * FSM handles the fault internally and transitions to FAULT_STATE,
     * returning EXIT_SUCCESS to the caller (fault is handled, not propagated). */
    drive_command(false);
    BatteryPack_FSM();

    /* Verify the FSM reached FAULT state and set the correct fault flags */
    TEST_ASSERT_EQUAL_UINT8(FAULT_ID, current_state.state_id);
    TEST_ASSERT_TRUE(Error_IsSet(&AIR_P.fault_register, FAULT_CONTACTOR_DRIVE_SENSE_MISMATCH));
    TEST_ASSERT_TRUE(Error_IsSet(&BatteryPackErrorRegister, FAULT_BPACK_TRANSITION_FAULT));
}

void test_BatteryPack_Fault_02_Precharge_Resistor_Open(void) {
    /**
     * Verify that if the precharge resistor is open (very high resistance), the vehicle
     * voltage never reaches the 95% threshold and the FSM detects a precharge timeout,
     * sets FAULT_BPACK_PRECHARGE_FAILURE, and transitions to the FAULT state.
     */
    CircuitSim_SetPrechargeResistance(1e12); /* Effectively open */

    drive_command(true);
    TEST_ASSERT_EQUAL(EXIT_SUCCESS, BatteryPack_FSM());
    TEST_ASSERT_EQUAL_UINT8(PRECHARGE_ID, current_state.state_id);

    /* Spin the FSM until it leaves PRECHARGE (timeout → FAULT) */
    do {
        sim_steps(1, 0.001);
        BatteryPack_FSM();
    } while (current_state.state_id == PRECHARGE_ID);

    /* Precharge timeout drives FSM into FAULT (via FAULT_STATE transition) */
    TEST_ASSERT_EQUAL_UINT8(FAULT_ID, current_state.state_id);
    TEST_ASSERT_TRUE(Error_IsSet(&BatteryPackErrorRegister, FAULT_BPACK_PRECHARGE_FAILURE));
    TEST_ASSERT_FALSE(Error_IsSet(&PRECHARGE.fault_register, FAULT_CONTACTOR_DRIVE_SENSE_MISMATCH));
}

void test_BatteryPack_Fault_03_Discharge_Resistor_Open(void) {
    /**
     * Verify that if the discharge resistor is open, the FSM detects a discharge timeout
     * when transitioning through DISCHARGE state and transitions to FAULT, setting
     * FAULT_BPACK_DISCHARGE_FAILURE.
     */
    CircuitSim_SetDischargeResistance(1e12); /* Effectively open */

    drive_command(true);
    sim_steps(7.0, 0.001); /* Simulate precharge and running */

    TEST_ASSERT_EQUAL_UINT8(RUNNING_ID, current_state.state_id);
    TEST_ASSERT_FALSE_MESSAGE(
        hasAnyFaults(), "Expected no faults during precharge and running with open discharge resistor"
    );

    /* Trigger shutdown → DISCHARGE */
    drive_command(false);

    /* Spin until FSM leaves DISCHARGE (timeout → FAULT) */
    do {
        sim_steps(1, 0.001);
        BatteryPack_FSM();
    } while (current_state.state_id == DISCHARGE_ID);

    TEST_ASSERT_EQUAL_UINT8(FAULT_ID, current_state.state_id);
    TEST_ASSERT_TRUE(Error_IsSet(&BatteryPackErrorRegister, FAULT_BPACK_DISCHARGE_FAILURE));
    TEST_ASSERT_FALSE(Error_IsSet(&DISCHARGE.fault_register, FAULT_CONTACTOR_DRIVE_SENSE_MISMATCH));
}

void test_BatteryPack_Fault_05_Contactor_Sense_Disconnected(void) {
    TEST_IGNORE_MESSAGE("Not implemented yet");
    // TODO : Implement broken-cable functionality in CircuitSim and test that FSM detects sense feedback mismatch
}
int main(void) {
    UNITY_BEGIN();

    RUN_TEST(test_BatteryPack_Init_Success);
    RUN_TEST(test_BatteryPack_Initial_State);
    RUN_TEST(test_BatteryPack_Safe_To_Precharge_Transition);
    RUN_TEST(test_BatteryPack_Precharge_To_Running_Transition);
    RUN_TEST(test_BatteryPack_Running_To_Discharge_Transition);
    RUN_TEST(test_BatteryPack_Normal_Cycle);
    RUN_TEST(test_BatteryPack_Shutdown_Uses_Discharge_Contactor);

    RUN_TEST(test_BatteryPack_Fault_01_AIR_P_Welded_After_First_Close);
    RUN_TEST(test_BatteryPack_Fault_02_Precharge_Resistor_Open);
    RUN_TEST(test_BatteryPack_Fault_03_Discharge_Resistor_Open);
    RUN_TEST(test_BatteryPack_Fault_05_Contactor_Sense_Disconnected);
    return UNITY_END();
}
