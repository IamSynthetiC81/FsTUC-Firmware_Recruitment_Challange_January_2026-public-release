#ifndef CONTACTOR_H
#define CONTACTOR_H

#include <stdint.h>
#include <stdbool.h>

#include "EEPROM.h"
#include "HAL_io.h"
#include "Errno.h"

/* =======================================================================
 * Contactor Error Bit and Structure Definitions
 * ====================================================================== */

#define FAULT_CONTACTOR_DRIVE_SENSE_MISMATCH		0
#define FAULT_CONTACTOR_STUCK_CLOSED				    1
#define FAULT_CONTACTOR_STUCK_OPEN				      2
#define FAULT_CONTACTOR_CYCLE_COUNT_OVERFLOW		3
#define FAULT_CONTACTOR_INVALID_CONFIG          4
#define FAULT_CONTACTOR_RUNTIME_OVERFLOW			  8
#define FAULT_CONTACTOR_EEPROM_WRITE_FAILURE		9
#define FAULT_CONTACTOR_EEPROM_READ_FAILURE		  10
#define FAULT_CONTACTOR_EEPROM_INVALID_ADDRESS	11

#define FAULT_CONTACTOR_HARD_MASK 0x000000FF

extern ErrorRegister_t ContactorErrorRegister;

/* =======================================================================
 * Contactor Data Structures and Enumerations
 * ====================================================================== */

typedef struct {
	uint32_t runtime;
	uint32_t accumulated_runtime;
	uint32_t cycle_count;

	uint16_t EEPROM_accumulated_runtime_address;
	bool EEPROM_track_runtime;

	uint16_t EEPROM_cycle_count_address;
	bool EEPROM_track_cycle_count;

} Contactor_PersistanceData_t;

typedef enum {
  CLOSED = 0,
  OPEN = 1
} ContactorState_t;

typedef enum {
  ACTIVE_LOW = 0, // Closed when drive pin is LOW
  ACTIVE_HIGH = 1 // Closed when drive pin is HIGH
} ContactorActiveLogic_t;

typedef struct {
  GPIO_Pin_t GPIO_Drive_pin;
  ContactorActiveLogic_t drive_logic;
  ContactorState_t normalState;

  GPIO_Pin_t GPIO_Sense_pin;
  ContactorActiveLogic_t sense_logic;
  ContactorState_t sense_normal_state ;

  ContactorState_t current_state;

  Contactor_PersistanceData_t persistance_data;
  ErrorRegister_t fault_register;


} Contactor_t;

/* =======================================================================
 * Contactor Function Prototypes
 * ====================================================================== */

/**
  * @brief Initializes the contactor structure and hardware.
  *
  * @param contactor Pointer to the Contactor_t structure to initialize.
  * @return EXIT_SUCCESS if initialization is successful, EXIT_FAILURE otherwise.
  *
  */
bool Contactor_Init(Contactor_t* contactor);

/**
  * @brief Sets the state of the contactor (OPEN or CLOSED).
  *
  * @param contactor Pointer to the Contactor_t structure.
  * @param state Desired state to set (OPEN or CLOSED).
  * @return EXIT_SUCCESS if the operation is successful, EXIT_FAILURE otherwise.
  */
bool Contactor_SetState(Contactor_t* contactor,ContactorState_t state);


/**
  * @brief Verifies and updates the current state of the contactor based on the sense pin reading.
  *
  * @param contactor Pointer to the Contactor_t structure.
  * @return ContactorState_t representing the current state of the contactor.
  *
  * @note If a fault is detected (e.g., drive and sense states do not match),
  *   the contactor's fault register will be updated accordingly. The function will 
  *   still return the current state based on the sense pin, but the caller should 
  *   check the fault register for any issues.
  */
ContactorState_t Contactor_GetState(Contactor_t* contactor);



#endif // CONTACTOR_H