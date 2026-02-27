#ifndef BATTERYPACK_H
#define BATTERYPACK_H

#include <stdio.h>
#include <stdlib.h>
#include "contactor.h"
#include "FSM.h"
#include "ADC.h"
#include "HAL_io.h"

/* =======================================================================
 * Battery Pack Error Bit and Structure Definitions
 * ====================================================================== */
#include "ErrorRegister.h"

#define FAULT_BPACK_UNDERVOLTAGE        0
#define FAULT_BPACK_OVERVOLTAGE         1
#define FAULT_BPACK_OVERCURRENT         2
#define FAULT_BPACK_VEHICLE_SIDE_HOT    3
#define FAULT_BPACK_AIR_P_FAILURE       4
#define FAULT_BPACK_AIR_N_FAILURE       5
#define FAULT_BPACK_PRECHARGE_FAILURE   6
#define FAULT_BPACK_DISCHARGE_FAILURE   7
#define FAULT_BPACK_SENSOR_FAULT_BIT    8
#define FAULT_BPACK_INVALID_TRANSITION  9
#define FAULT_BPACK_TRANSITION_FAULT   10

#define FAULT_BPACK_ADC_FAILURE_BIT 11

#define FAULT_BPACK_HARD_MASK 0xFFFFFFFF

extern ErrorRegister_t BatteryPackErrorRegister;

static const uint32_t DISCHARGE_TIMEOUT_MS = 5000; // 5 seconds timeout for discharge
static const uint32_t PRECHARGE_TIMEOUT_MS = 5000; // 5 seconds timeout for precharge

static const uint16_t DISCHARGE_VOLTAGE_THRESHOLD_ADC = (uint16_t)((60.0 * 4095.0) / 600.0); // Example threshold for discharge state
static const float PRECHARGE_VOLTAGE_RATIO = 0.95; // 95% of battery voltage as threshold for precharge state

/* ============================================================================
 * External Hardware Definitions
 * ========================================================================== */
extern Contactor_t AIR_P;
extern Contactor_t AIR_N;
extern Contactor_t PRECHARGE;
extern Contactor_t DISCHARGE;

extern ADC_Handle_t g_adc;

/* ============================================================================
 * GPIO Pin Definitions
 * ========================================================================== */

/* ================================~ Inputs ~================================ */
extern GPIO_Pin_t drive_command_pin;
extern GPIO_Pin_t AIR_P_AUX_pin;
extern GPIO_Pin_t AIR_N_AUX_pin;
extern GPIO_Pin_t PRECHARGE_AUX_pin;
extern GPIO_Pin_t DISCHARGE_AUX_pin;

/* ================================~ Outputs ~=============================== */
extern GPIO_Pin_t AIR_P_drive_pin;
extern GPIO_Pin_t AIR_N_drive_pin;
extern GPIO_Pin_t PRECHARGE_drive_pin;
extern GPIO_Pin_t DISCHARGE_drive_pin;

/* ===============================~ Indicators ~============================== */
extern GPIO_Pin_t fault_indicator_pin;
extern GPIO_Pin_t ready_to_race_indicator_pin;
extern GPIO_Pin_t idle_indicator_pin;

/* ============================================================================
 * Battery Pack Control API
 * ========================================================================== */

extern FSM_State_t current_state;

bool BatteryPack_Init(void);

bool BatteryPack_FaultHandler(void);

bool BatteryPack_FSM(void);

#endif  // BATTERYPACK_H
