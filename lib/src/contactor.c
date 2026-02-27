#include "contactor.h"
#include <assert.h>
#include <stdlib.h>

#include "BatteryPack.h"

/**
 * @brief Global ContactorErrorRegister instance
 */
ErrorRegister_t ContactorErrorRegister = {
    .peripheralName = "Contactor",
    .errorReg = 0
};

static ContactorState_t Contactor_GetPhysicalState(const Contactor_t* contactor) {
	if (!contactor) {
		Error_Set(&PROGRAM_ERRORS, ERROR_INVALID_POINTER);
		fprintf(stderr, "Contactor_GetPhysicalState: Invalid contactor pointer\n");
		exit(EXIT_FAILURE);
	}

	const bool pin_is_high = (HAL_GPIO_ReadPin(contactor->GPIO_AUX_pin));

	ContactorState_t aux_physical_state;
	if (contactor->aux_logic == ACTIVE_HIGH) {
		aux_physical_state = pin_is_high ? CLOSED : OPEN;
	} else {
		aux_physical_state = pin_is_high ? OPEN : CLOSED;
	}

	return aux_physical_state;
}

/**
 * @brief Check if the sensed pin state matches the commanded state
 * @param contactor
 * @param commanded_state The desired physical state of the MAIN contact (OPEN or CLOSED)
 * @return true if sense pin matches the expected physical state of the auxiliary
 */
static bool aux_pin_valid_state_after_command(const Contactor_t* contactor, const ContactorState_t commanded_state) {
	const ContactorState_t aux_physical_state = Contactor_GetPhysicalState(contactor);

	// 3. Determine what the AUX physical state SHOULD be
	// When the main contact is in its normal state, the aux is always in its own normal state.
	// When the main contact is energized (not normal), the aux inverts from its normal state.
	// This relationship is independent of whether the contacts are "in phase" with each other.
	const bool main_is_normal = (commanded_state == contactor->main_normal_state);

	ContactorState_t expected_aux_state;
	if (main_is_normal) {
		expected_aux_state = contactor->aux_normal_state;
	} else {
		expected_aux_state = (contactor->aux_normal_state == OPEN) ? CLOSED : OPEN;
	}

	// 4. Comparison
	return (aux_physical_state == expected_aux_state);
}

static void Contactor_PullFromEEPROM(Contactor_t* contactor) {
	const bool track_cycle_count = contactor->persistance_data.EEPROM_track_cycle_count;
	const bool track_runtime = contactor->persistance_data.EEPROM_track_runtime;

	uint32_t stored_count = 0xFFFFFFFF;
	uint32_t stored_runtime = 0xFFFFFFFF;

	if (track_cycle_count) {
		const uint16_t stored_address = contactor->persistance_data.EEPROM_cycle_count_address;
		if (stored_address == (uint16_t)-1) {
			fprintf(stderr, "Contactor_Init: EEPROM tracking for cycle count enabled but no valid address provided\n");
			Error_Set(&contactor->fault_register,FAULT_CONTACTOR_EEPROM_INVALID_ADDRESS);
		} else {
			const bool success = EEPROM_Read(stored_address, (uint8_t*)&stored_count, sizeof(stored_count));
			if (!success) {
				fprintf(stderr, "Contactor_Init: Failed to read cycle count from EEPROM\n");
				Error_Set(&contactor->fault_register,FAULT_CONTACTOR_EEPROM_READ_FAILURE);
			}
		}
	}

	contactor->persistance_data.cycle_count = stored_count == 0xFFFFFFFF ? 0 : stored_count;

	if (track_runtime) {
		const uint16_t stored_address = contactor->persistance_data.EEPROM_accumulated_runtime_address;

		if (stored_address == (uint16_t)-1) {
			fprintf(stderr, "Contactor_Init: EEPROM accumulated runtime address not set\n");
			Error_Set(&contactor->fault_register,FAULT_CONTACTOR_EEPROM_INVALID_ADDRESS);
			// no hard fault
		} else {
			const bool success = EEPROM_Read(stored_address, (uint8_t*)&stored_runtime, sizeof(stored_runtime));
			if (!success) {
				fprintf(stderr, "Contactor_Init: Failed to read accumulated runtime from EEPROM\n");
				Error_Set(&contactor->fault_register,FAULT_CONTACTOR_EEPROM_READ_FAILURE);
			}
		}

	}

	contactor->persistance_data.accumulated_runtime = stored_runtime == 0xFFFFFFFF ? 0 : stored_runtime;
}

static bool isHardFault(const Contactor_t* contactor) {
	const bool has_hard_fault = (Error_Match(&contactor->fault_register, FAULT_CONTACTOR_HARD_MASK )) != 0;

	return has_hard_fault;
}

bool Contactor_Init(Contactor_t* contactor) {
	if ( !contactor) {
		Error_Set(&PROGRAM_ERRORS, ERROR_INVALID_POINTER);
		fprintf(stderr, "Contactor_Init: Invalid contactor pointer\n");
		return EXIT_FAILURE;
	}
		
	// 1. VALIDATION: Check for illegal electrical configs
    // Active High needs a Pull-Down to be safe; Active Low needs a Pull-Up.
    if (
    	(contactor->aux_logic == ACTIVE_HIGH && contactor->GPIO_AUX_pin.pull == GPIO_PULL_UP) ||
		(contactor->aux_logic == ACTIVE_LOW && contactor->GPIO_AUX_pin.pull == GPIO_PULL_DOWN) ||
		(contactor->drive_logic == ACTIVE_HIGH && contactor->GPIO_Drive_pin.pull == GPIO_PULL_UP) ||
		(contactor->drive_logic == ACTIVE_LOW && contactor->GPIO_Drive_pin.pull == GPIO_PULL_DOWN)
	) {
	    Error_Set(&contactor->fault_register, FAULT_CONTACTOR_INVALID_CONFIG);
    	return EXIT_FAILURE;
    }

	// Initialize GPIO pins
	HAL_GPIO_Init(contactor->GPIO_Drive_pin);
	HAL_GPIO_Init(contactor->GPIO_AUX_pin);

	// Initialize drive pin to de-energized state (OPEN)
	const bool initial_drive_level = (contactor->drive_logic == ACTIVE_HIGH) ? false : true;
	HAL_GPIO_WritePin(contactor->GPIO_Drive_pin, initial_drive_level);

	/* == PERSISTENCE SETUP == */
	Contactor_PullFromEEPROM(contactor);

	/* == FAULT CHECKING == */
	if (isHardFault(contactor)) {
		return EXIT_FAILURE;
	}
	if (Error_IsSet(&contactor->fault_register, FAULT_CONTACTOR_RUNTIME_OVERFLOW)) {
		fprintf(stderr, "Contactor_Init: Runtime overflow detected during initialization\n");
		return EXIT_FAILURE;
	}
	if (Error_IsSet(&contactor->fault_register, FAULT_CONTACTOR_CYCLE_COUNT_OVERFLOW)) {
		fprintf(stderr, "Contactor_Init: Cycle count overflow detected during initialization\n");
		return EXIT_FAILURE;
	}

    contactor->current_state = OPEN;
    return Contactor_SetState(contactor, OPEN);
}

bool Contactor_SetState(Contactor_t* contactor,const ContactorState_t state) {
	if (!contactor) {
		Error_Set(&PROGRAM_ERRORS, ERROR_INVALID_POINTER);
		fprintf(stderr, "Contactor_SetState: Invalid contactor pointer\n");
		return EXIT_FAILURE;
	}

	// Track previous state for cycle counting
	const ContactorState_t previous_state = contactor->current_state;
	
	// Determine drive pin level based on desired state, drive logic and normal state
	bool drive_level;
	if (state == CLOSED) {
		drive_level = contactor->drive_logic == ACTIVE_HIGH ? true : false;
	} else { // state == OPEN
		drive_level = contactor->drive_logic == ACTIVE_HIGH ? false : true;
	}
	
	if (!HAL_GPIO_WritePin(contactor->GPIO_Drive_pin, drive_level)) {
		return EXIT_FAILURE;
	}

	HAL_Delay_ms(150); // Wait for contactor to physically move (example travel time)

	// Update current state based on sense pin reading and check for faults
	contactor->current_state = state;
	contactor->current_state = Contactor_GetState(contactor);
	if (isHardFault(contactor)) return EXIT_FAILURE;

	// Increment and persist cycle count only if we successfully changed state
	if (previous_state != contactor->current_state){
        if (contactor->persistance_data.cycle_count++ == 0xFFFFFFFF) {
			Error_Set(&contactor->fault_register, FAULT_CONTACTOR_CYCLE_COUNT_OVERFLOW);
		}
	}
	
	return EXIT_SUCCESS	;
}

ContactorState_t Contactor_GetState(Contactor_t* contactor) {
    if (!contactor) {
	    Error_Set(&PROGRAM_ERRORS, ERROR_INVALID_POINTER);
		fprintf(stderr, "Contactor_GetState: Invalid contactor pointer\n");
		exit(EXIT_FAILURE);
    }

    if (!aux_pin_valid_state_after_command(contactor, contactor->current_state)) {
		Error_Set(&contactor->fault_register, FAULT_CONTACTOR_DRIVE_AUX_MISMATCH);
    	const ContactorState_t aux_physical_state = Contactor_GetPhysicalState(contactor);

    	const bool inPhase = (contactor->main_normal_state == contactor->aux_normal_state);
    	return (inPhase) ? aux_physical_state : (aux_physical_state == OPEN ? CLOSED : OPEN);
	}

	return contactor->current_state;
}