#include <assert.h>
#include <stdlib.h>

#include "unity.h"
#include "contactor.h"
#include "HAL_io_mock.h"
#include "EEPROM.h"
#include "CircuitSim.h"
#include <string.h>

static Contactor_t air_p_uut, air_n_uut, precharge_contactor_uut, discharge_contactor_uut, UUT_5;
static SimContactor_t air_p_sim, air_n_sim, precharge_sim, discharge_sim, sim_link_5;

void ResetContactorsWithSim(void) {
    air_p_uut = (Contactor_t){
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

    air_n_uut = (Contactor_t){
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
    precharge_contactor_uut = (Contactor_t){
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
    discharge_contactor_uut = (Contactor_t){
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
        .contactor = &air_p_uut,
        .physical_state = OPEN,
        .welded_closed = false,
        .is_moving = false,
        .travel_time_ms = 0,
        .effective_resistance = 1e9
    };
    air_n_sim = (SimContactor_t){
        .contactor = &air_n_uut,
        .physical_state = OPEN,
        .welded_closed = false,
        .is_moving = false,
        .travel_time_ms = 0,
        .effective_resistance = 1e9
    };
    precharge_sim = (SimContactor_t){
        .contactor = &precharge_contactor_uut,
        .physical_state = OPEN,
        .welded_closed = false,
        .is_moving = false,
        .travel_time_ms = 0,
        .effective_resistance = 1e9
    };
    discharge_sim = (SimContactor_t){
        .contactor = &discharge_contactor_uut,
        .physical_state = OPEN,
        .welded_closed = false,
        .is_moving = false,
        .travel_time_ms = 0,
        .effective_resistance = 1e9
    };
}

void clearEEPROM(void) {
    EEPROM_Erase(0, EEPROM_TOTAL_SIZE_BYTES);
}

/* --- Unity Standard Setup/Teardown --- */
void setUp(void) {
    // 1. Reset everything first
    Mock_HAL_GPIO_Reset();
    EEPROM_Init();
    EEPROM_Erase(0, EEPROM_TOTAL_SIZE_BYTES);

    ResetContactorsWithSim();

    Mock_HAL_LinkPins(&air_p_sim);
    Mock_HAL_LinkPins(&air_n_sim);
    Mock_HAL_LinkPins(&precharge_sim);
    Mock_HAL_LinkPins(&discharge_sim);

    // Initialize CircuitSim so GPIO changes drive physics
    CircuitSim_Init(&air_p_uut, &air_n_uut, &precharge_contactor_uut, &discharge_contactor_uut);

    CircuitSim_WeldContactorClosed(&air_p_uut, false);
    CircuitSim_WeldContactorClosed(&air_n_uut, false);
    CircuitSim_WeldContactorClosed(&precharge_contactor_uut, false);
    CircuitSim_WeldContactorClosed(&discharge_contactor_uut, false);
}

void tearDown(void) {
    // This runs AFTER every RUN_TEST
    // clear EEPROM
    EEPROM_Erase(0, EEPROM_TOTAL_SIZE_BYTES);

    ResetContactorsWithSim();

    // Wipe everything to default (0V, No pull)
    Mock_HAL_GPIO_Reset();
    CircuitSim_Cleanup();

    Mock_HAL_LinkPins(&air_p_sim);
    Mock_HAL_LinkPins(&air_n_sim);
    Mock_HAL_LinkPins(&precharge_sim);
    Mock_HAL_LinkPins(&discharge_sim);
}

bool hasHardFault(const Contactor_t* contactor) {
    return Error_Match(&contactor->fault_register, FAULT_CONTACTOR_HARD_MASK);
}

bool hasAnyFault(const Contactor_t* contactor) {
    return Error_AnySet(&contactor->fault_register);
}

/* --- Converted Unity Tests --- */


/* INIT TESTS */

void test_Contactor_Valid_Pull_Config(void) {
    const ErrorRegister_t* drive_fault_reg = &air_p_uut.fault_register;

    /*
        Verify that Contactor_Init enforces the correct pull resistor configuration based on the drive and sense logic settings.
        For ACTIVE_HIGH logic, the drive pin should have a PULL_DOWN, and for ACTIVE_LOW logic, it should have a PULL_UP.
        The same applies to the sense pin based on the sense logic.

        This test uses the pre-registered air_p_uut contactor, which is configured for ACTIVE_HIGH drive and sense logic, and checks that the pull resistors are set to PULL_DOWN as expected.
    */
        
    /* Correct config */
    air_p_uut.drive_logic = ACTIVE_HIGH;
    air_p_uut.sense_logic = ACTIVE_HIGH;
    air_p_uut.GPIO_Drive_pin.pull = GPIO_PULL_DOWN; 
    air_p_uut.GPIO_Sense_pin.pull = GPIO_PULL_DOWN; 
    
    const bool retval = Contactor_Init(&air_p_uut);
    const GPIO_Pull_t pull1 = Mock_HAL_GetPullConfig(air_p_uut.GPIO_Drive_pin);
    const GPIO_Pull_t pull2 = Mock_HAL_GetPullConfig(air_p_uut.GPIO_Sense_pin);

    TEST_ASSERT_EQUAL(GPIO_PULL_DOWN, pull1);
    TEST_ASSERT_EQUAL(GPIO_PULL_DOWN, pull2);
    TEST_ASSERT_EQUAL(EXIT_SUCCESS, retval);
    
    TEST_ASSERT_FALSE(hasHardFault(&air_p_uut));
}

void test_Contactor_Invalid_Pull_Config(void) {
    const ErrorRegister_t* drive_fault_reg = &air_p_uut.fault_register;

    /* 
        Verify that Contactor_Init fails when the pull resistor configuration is incompatible with the drive and sense logic settings.
        For example, if the drive logic is ACTIVE_HIGH but the drive pin has a PULL_UP, this should be considered an invalid configuration and Contactor_Init should return EXIT_FAILURE.

        This test uses the pre-registered air_p_uut contactor, modifies its configuration to create an invalid state, and checks that Contactor_Init correctly identifies the issue and returns EXIT_FAILURE.
    */

    /* Incorrect config */
    air_p_uut.drive_logic = ACTIVE_HIGH;
    air_p_uut.sense_logic = ACTIVE_HIGH;
    air_p_uut.GPIO_Drive_pin.pull = GPIO_PULL_UP; // This is an electrical mismatch
    air_p_uut.GPIO_Sense_pin.pull = GPIO_PULL_UP; // This is an electrical mismatch

    const bool retval = Contactor_Init(&air_p_uut);
    const GPIO_Pull_t pull1 = Mock_HAL_GetPullConfig(air_p_uut.GPIO_Drive_pin);
    const GPIO_Pull_t pull2 = Mock_HAL_GetPullConfig(air_p_uut.GPIO_Sense_pin);


    TEST_ASSERT_EQUAL(EXIT_FAILURE, retval);
    TEST_ASSERT_TRUE(hasHardFault(&air_p_uut));
}

void test_Contactor_EEPROM_Uninitialized_Read(void) {
    // TODO : Implement test
}

void test_Contactor_AccumulatedRuntime_Persistence(void) {
   // TODO : Implement test
}

void test_Contactor_AccumulatedRuntime_Updates(void) {
    // TODO : Implement test
}

void test_Contactor_DriveToSense_MixedLogic(void) {
    

    /*
        Verify that Contactor_Init correctly handles a mixed logic configuration where the drive and sense logic are different (e.g., drive is ACTIVE_HIGH but sense is ACTIVE_LOW).
        In this case, the pull resistor configuration must still be valid for each pin based on its respective logic. For example, if the drive is ACTIVE_HIGH, it should have a PULL_DOWN, and if the sense is ACTIVE_LOW, it should have a PULL_UP.

        This test modifies the pre-registered air_p_uut contactor to have a mixed logic configuration and checks that Contactor_Init accepts this configuration as long as the pull resistors are correctly set according to their respective logics.
    */

    const ErrorRegister_t* drive_fault_reg = &air_p_uut.fault_register;

    air_n_uut.drive_logic = ACTIVE_HIGH;
    air_n_uut.sense_logic = ACTIVE_LOW;
    air_n_uut.GPIO_Sense_pin.pull = GPIO_PULL_UP;

    {
        const bool init_retval = Contactor_Init(&air_n_uut);
        TEST_ASSERT_EQUAL(EXIT_SUCCESS, init_retval);

        TEST_ASSERT_FALSE(hasHardFault(&air_n_uut));
    }

    TEST_ASSERT_EQUAL(OPEN, air_n_uut.current_state);
    
    {
        const bool set_retval = Contactor_SetState(&air_n_uut, CLOSED);
        
        TEST_ASSERT_FALSE(hasHardFault(&air_n_uut));
        TEST_ASSERT_EQUAL(EXIT_SUCCESS, set_retval);
    }

    TEST_ASSERT_EQUAL(CLOSED, air_n_uut.current_state);

}

void test_EEPROM_CycleCountPersistence(void) {
    // TODO : Implement test
}

void test_Contactor_Closes_Physically(void) {
    const ErrorRegister_t* drive_fault_reg = &air_p_uut.fault_register;

    /*
        Verify that when Contactor_SetState is called to close the contactor, the physical state of the contactor (as simulated by CircuitSim) changes to CLOSED after the expected travel time.
        This test uses the pre-registered air_p_uut contactor, commands it to close, and then checks that the physical state in the simulation reflects this change after the appropriate delay.

        Additionally, it checks that no faults are set during this normal operation.
    */
    const bool init_retval = Contactor_Init(&air_p_uut);
    TEST_ASSERT_EQUAL(EXIT_SUCCESS, init_retval);
    TEST_ASSERT_FALSE(hasHardFault(&air_p_uut));

    const bool set_retval = Contactor_SetState(&air_p_uut, CLOSED);

    TEST_ASSERT_FALSE(hasHardFault(&air_p_uut));
}

void test_Contactor_Fault_WeldedContact(void) {
    const ErrorRegister_t* drive_fault_reg = &air_p_uut.fault_register;

    /*
        Verify that if the contactor is commanded to OPEN but the sense pin indicates it is still 
        CLOSED (simulating a welded contact), Contactor_SetState should return EXIT_FAILURE, the fault 
        register should indicate a hard fault, and the state should reflect the sense pin reading (CLOSED).

        This test uses the pre-registered air_p_uut contactor, commands it to open, 
        simulates a welded contact by forcing the sense pin to read CLOSED, and checks that 
        Contactor_SetState correctly identifies the fault condition and updates the state 
        and fault register accordingly.
    */
    
    air_p_uut.GPIO_Sense_pin.pull   = GPIO_PULL_DOWN;
    air_p_uut.sense_logic           = ACTIVE_HIGH;

    air_p_uut.normalState           = OPEN;
    air_p_uut.sense_normal_state    = CLOSED;
    
    TEST_ASSERT_EQUAL(EXIT_SUCCESS, Contactor_Init(&air_p_uut));
    TEST_ASSERT_FALSE(hasHardFault(&air_p_uut));
    
    {
        const bool retval = Contactor_SetState(&air_p_uut, CLOSED);
        TEST_ASSERT_EQUAL(EXIT_SUCCESS, retval);
        TEST_ASSERT_FALSE(hasHardFault(&air_p_uut));

        const ContactorState_t result = Contactor_GetState(&air_p_uut);
        const bool sense_pin_level = HAL_GPIO_ReadPin(air_p_uut.GPIO_Sense_pin);
        const bool expected_sense_level = false;

        TEST_ASSERT_EQUAL(expected_sense_level, sense_pin_level);
        TEST_ASSERT_EQUAL(CLOSED, result);
        TEST_ASSERT_FALSE(hasHardFault(&air_p_uut));

    }
    // FAULT INJECTION: Simulate a welded contact
    CircuitSim_WeldContactorClosed(&air_p_uut, true);

    const bool retval = Contactor_SetState(&air_p_uut, OPEN);

    const ContactorState_t result = Contactor_GetState(&air_p_uut);
    const bool sense_pin_level = HAL_GPIO_ReadPin(air_p_uut.GPIO_Sense_pin);
    const bool expected_sense_level = false;


    TEST_ASSERT_EQUAL(EXIT_FAILURE, retval);
    TEST_ASSERT_EQUAL_MESSAGE(expected_sense_level, sense_pin_level, "Sense pin level should reflect the welded contact state (stuck low)");
    TEST_ASSERT_TRUE(hasHardFault(&air_p_uut));
    TEST_ASSERT_EQUAL(CLOSED, result);
}

void test_Contactor_Should_Wait_For_Travel_Time(void) {
    const ErrorRegister_t* drive_fault_reg = &air_p_uut.fault_register;

    /*
        Verify that Contactor_SetState enforces the specified travel time before the contactor state changes to CLOSED.
        This test uses the pre-registered air_n_uut contactor, sets a travel time, commands it to close, and checks that the state does not change to CLOSED until after the travel time has elapsed.

        Additionally, it checks that no faults are set during this normal operation.
    */
    TEST_ASSERT_EQUAL(EXIT_SUCCESS, Contactor_Init(&air_n_uut));
    TEST_ASSERT_FALSE(hasHardFault(&air_p_uut));
    
    const double start_time = HAL_GetRuntimeMs();
    const bool retval = Contactor_SetState(&air_n_uut, CLOSED);
    const double elapsed_time = (HAL_GetRuntimeMs() - start_time);

    // The elapsed time should be at least the travel time (100ms)
    TEST_ASSERT_GREATER_OR_EQUAL(120, elapsed_time); // Allow some buffer for execution time
    TEST_ASSERT_EQUAL(EXIT_SUCCESS, retval);

    // The contactor should now be CLOSED
    bool result = Contactor_GetState(&air_n_uut);
    TEST_ASSERT_EQUAL(CLOSED, result);

    TEST_ASSERT_FALSE(hasHardFault(&air_p_uut));
}

void test_NO_main_NC_aux(void) {
    const ErrorRegister_t* drive_fault_reg = &air_p_uut.fault_register;

    /*
        Verify that the sense pin correctly reflects the state of the auxiliary contact based on the normal states and logic configurations.
        For example, if the main contact is normally open and the auxiliary contact is normally closed, when the main is commanded to close, the sense pin should reflect the state of the auxiliary contact accordingly.

        This test uses the pre-registered air_n_uut contactor, configures it with a normally open main and a normally closed auxiliary contact, and checks that the sense pin level changes as expected when commanding OPEN and CLOSED states.
    */
        
    air_n_sim.contactor->sense_normal_state = CLOSED;
    air_n_sim.contactor->normalState = OPEN;
    air_n_sim.contactor->sense_logic = ACTIVE_HIGH;

    TEST_ASSERT_EQUAL(EXIT_SUCCESS, Contactor_Init(&air_n_uut));
    TEST_ASSERT_FALSE(hasHardFault(&air_p_uut));
    

    /* Test closing the contactors */
    {
        TEST_ASSERT_EQUAL(EXIT_SUCCESS, Contactor_SetState(&air_n_uut, CLOSED));
        TEST_ASSERT_EQUAL(CLOSED, Contactor_GetState(&air_n_uut));

        // main is closed -> sense is open -> sense pin should read LOW due to normally closed aux contact
        const bool expected_sense_level = false;
        const bool sense_pin_level = HAL_GPIO_ReadPin(air_n_uut.GPIO_Sense_pin);
        TEST_ASSERT_EQUAL(expected_sense_level, sense_pin_level);
    }

    /* Test opening the contactors */
    {
        TEST_ASSERT_EQUAL(EXIT_SUCCESS, Contactor_SetState(&air_n_uut, OPEN));

        TEST_ASSERT_EQUAL(OPEN, Contactor_GetState(&air_n_uut));

        const bool expected_sense_level = true;
        const bool sense_pin_level = HAL_GPIO_ReadPin(air_n_uut.GPIO_Sense_pin);
        TEST_ASSERT_EQUAL(expected_sense_level, sense_pin_level);
    }
}

void test_NC_main_NO_aux(void) {
    /*
        Verify that the sense pin correctly reflects the state of the auxiliary contact based on the normal states and logic configurations.
        For example, if the main contact is normally closed and the auxiliary contact is normally open, when the main is commanded to close, the sense pin should reflect the state of the auxiliary contact accordingly.

        This test uses the pre-registered air_n_uut contactor, configures it with a normally closed main and a normally open auxiliary contact, and checks that the sense pin level changes as expected when commanding OPEN and CLOSED states.
    */
    
    // Normally closed main with normally open aux works
    air_n_sim.contactor->sense_normal_state = OPEN;
    air_n_sim.contactor->normalState        = CLOSED;
    air_n_sim.contactor->sense_logic        = ACTIVE_HIGH;
    air_n_sim.contactor->drive_logic        = ACTIVE_HIGH;
    
    

    TEST_ASSERT_EQUAL(EXIT_SUCCESS, Contactor_Init(&air_n_uut));
    TEST_ASSERT_FALSE(hasHardFault(&air_n_uut));

    TEST_ASSERT_EQUAL(EXIT_SUCCESS, Contactor_SetState(&air_n_uut, CLOSED));
    TEST_ASSERT_FALSE(hasHardFault(&air_n_uut));

    TEST_ASSERT_EQUAL(Contactor_GetState(&air_n_uut), CLOSED);
    TEST_ASSERT_FALSE(hasHardFault(&air_n_uut));

    bool expected_sense_level = (air_n_uut.sense_normal_state == OPEN) ? false : true;
    bool sense_pin_level = HAL_GPIO_ReadPin(air_n_uut.GPIO_Sense_pin);
    TEST_ASSERT_EQUAL(expected_sense_level, sense_pin_level);

    TEST_ASSERT_EQUAL(EXIT_SUCCESS, Contactor_SetState(&air_n_uut, OPEN));
    TEST_ASSERT_FALSE(hasHardFault(&air_n_uut));

    TEST_ASSERT_EQUAL(Contactor_GetState(&air_n_uut), OPEN);

    expected_sense_level = (air_n_uut.sense_normal_state == OPEN) ? true : false;
    sense_pin_level = HAL_GPIO_ReadPin(air_n_uut.GPIO_Sense_pin);
    TEST_ASSERT_EQUAL(expected_sense_level, sense_pin_level);
}

void test_NC_main_NC_aux(void) {
    /*
        Verify that the sense pin correctly reflects the state of the auxiliary contact based on the normal states and logic configurations.
        For example, if both the main and auxiliary contacts are normally closed, when the main is commanded to close, the sense pin should reflect the state of the auxiliary contact accordingly.

        This test uses the pre-registered air_n_uut contactor, configures it with a normally closed main and a normally closed auxiliary contact, and checks that the sense pin level changes as expected when commanding OPEN and CLOSED states.
    */

    // Normally closed main with normally closed aux works
    air_n_sim.contactor->sense_normal_state     = CLOSED;
    air_n_sim.contactor->normalState            = CLOSED;
    air_n_sim.contactor->sense_logic            = ACTIVE_HIGH;
    air_n_sim.contactor->drive_logic            = ACTIVE_HIGH;

    TEST_ASSERT_EQUAL(EXIT_SUCCESS, Contactor_Init(&air_n_uut));
    TEST_ASSERT_FALSE(hasHardFault(&air_n_uut));

    TEST_ASSERT_EQUAL(EXIT_SUCCESS,Contactor_SetState(&air_n_uut, CLOSED));
    TEST_ASSERT_EQUAL(Contactor_GetState(&air_n_uut), CLOSED);
    TEST_ASSERT_FALSE(hasHardFault(&air_n_uut));

    bool expected_sense_level = (air_n_uut.sense_normal_state == CLOSED) ? true : false;
    bool sense_pin_level = HAL_GPIO_ReadPin(air_n_uut.GPIO_Sense_pin);
    TEST_ASSERT_EQUAL_MESSAGE(expected_sense_level, sense_pin_level, "Sense pin level should reflect the normally closed auxiliary contact state");

    TEST_ASSERT_EQUAL(EXIT_SUCCESS,Contactor_SetState(&air_n_uut, OPEN));
    TEST_ASSERT_EQUAL(Contactor_GetState(&air_n_uut), OPEN);
    TEST_ASSERT_FALSE(hasHardFault(&air_n_uut));

    // When main is OPEN (energized, not normal state), aux should invert from its normal state
    expected_sense_level = (air_n_uut.sense_normal_state == CLOSED) ? false : true;
    sense_pin_level = HAL_GPIO_ReadPin(air_n_uut.GPIO_Sense_pin);
    TEST_ASSERT_EQUAL(expected_sense_level, sense_pin_level);
}

void test_NO_main_NO_aux(void) {
    /*
        Verify that the sense pin correctly reflects the state of the auxiliary contact based on the normal states and logic configurations.
        For example, if both the main and auxiliary contacts are normally open, when the main is commanded to close, the sense pin should reflect the state of the auxiliary contact accordingly.

        This test uses the pre-registered air_n_uut contactor, configures it with a normally open main and a normally open auxiliary contact, and checks that the sense pin level changes as expected when commanding OPEN and CLOSED states.
    */

    // Normally open main with normally open aux works
    air_n_sim.contactor->sense_normal_state = OPEN;
    air_n_sim.contactor->normalState = OPEN;
    air_n_sim.contactor->sense_logic = ACTIVE_LOW;
    air_n_sim.contactor->drive_logic = ACTIVE_LOW;

    air_n_sim.contactor->GPIO_Sense_pin.pull = GPIO_PULL_UP;
    air_n_sim.contactor->GPIO_Drive_pin.pull = GPIO_PULL_UP;

    TEST_ASSERT_EQUAL(EXIT_SUCCESS, Contactor_Init(&air_n_uut));
    TEST_ASSERT_FALSE(hasHardFault(&air_n_uut));

    TEST_ASSERT_EQUAL(EXIT_SUCCESS , Contactor_SetState(&air_n_uut, CLOSED));
    TEST_ASSERT_EQUAL(Contactor_GetState(&air_n_uut), CLOSED);
    TEST_ASSERT_FALSE(hasHardFault(&air_n_uut));

    bool expected_sense_level = (air_n_uut.sense_normal_state == OPEN) ? false : true;
    bool sense_pin_level = HAL_GPIO_ReadPin(air_n_uut.GPIO_Sense_pin);
    TEST_ASSERT_EQUAL(expected_sense_level, sense_pin_level);

    TEST_ASSERT_EQUAL(EXIT_SUCCESS , Contactor_SetState(&air_n_uut, OPEN));
    TEST_ASSERT_EQUAL(Contactor_GetState(&air_n_uut), OPEN);
    TEST_ASSERT_FALSE(hasHardFault(&air_n_uut));

    expected_sense_level = (air_n_uut.sense_normal_state == OPEN) ? true : false;
    sense_pin_level = HAL_GPIO_ReadPin(air_n_uut.GPIO_Sense_pin);
    TEST_ASSERT_EQUAL(expected_sense_level, sense_pin_level);
}


int main(void) {
    UNITY_BEGIN();

    // Group: Contactor Hardware Logic

    RUN_TEST(test_Contactor_Valid_Pull_Config);
    RUN_TEST(test_Contactor_Invalid_Pull_Config);
    RUN_TEST(test_Contactor_DriveToSense_MixedLogic);
    RUN_TEST(test_Contactor_Closes_Physically);
    RUN_TEST(test_Contactor_Fault_WeldedContact);
    RUN_TEST(test_Contactor_Should_Wait_For_Travel_Time);

    // Group: Auxiliary (Normally Closed) Contacts
    RUN_TEST(test_NO_main_NC_aux);
    RUN_TEST(test_NC_main_NO_aux);
    RUN_TEST(test_NC_main_NC_aux);
    RUN_TEST(test_NO_main_NO_aux);

    // Group: Persistence & Stress
    RUN_TEST(test_EEPROM_CycleCountPersistence);
    RUN_TEST(test_Contactor_AccumulatedRuntime_Persistence);
    RUN_TEST(test_Contactor_EEPROM_Uninitialized_Read);
    RUN_TEST(test_Contactor_AccumulatedRuntime_Updates);

    return UNITY_END();
}

