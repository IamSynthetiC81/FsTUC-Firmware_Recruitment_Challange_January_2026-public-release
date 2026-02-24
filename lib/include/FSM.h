#include <stdlib.h>

/********************
 * Battery Pack FSM
 * 
 * This FSM manages the state of the battery pack contactors based on system conditions.
 * It transitions between SAFE, PRECHARGE, RUNNING, DISCHARGE, and FAULT states.
 * 
 * The FSM should be called periodically (e.g., in the main loop) to evaluate conditions
 * and update contactor states accordingly.
 * 
 * State Descriptions:
 * - SAFE: All contactors open, waiting for start command
 * - PRECHARGE: Precharge contactor closed, monitoring voltage rise
 * - RUNNING: Main contactors closed, normal operation
 * - DISCHARGE: Main contactors closed, discharge contactor closed for regenerative braking
 * - FAULT: All contactors open, fault condition detected
 * 
 * @author FsTUC Recruitment Team
 * @date January 2026
 ********************/

#ifndef FSM_H
#define FSM_H

#define TRANSITION_BUSY 0
#define TRANSITION_COMPLETE 1
#define TRANSITION_FAILED -1

#include "contactor.h"

typedef enum {
    SAFE_ID = 0,
    PRECHARGE_ID = 1,
    RUNNING_ID = 2,
    DISCHARGE_ID = 3,
    FAULT_ID = 4
} FSM_id_t;

typedef struct {
    ContactorState_t air_p_state;
    ContactorState_t air_n_state;
    ContactorState_t precharge_state;
    ContactorState_t discharge_state;
    FSM_id_t state_id;
    uint32_t timestamp_ms; // Timestamp of when the state was entered, for timing checks
} FSM_State_t;

extern FSM_State_t SAFE_STATE;
extern FSM_State_t PRECHARGE_STATE;
extern FSM_State_t RUNNING_STATE;
extern FSM_State_t DISCHARGE_STATE;
extern FSM_State_t FAULT_STATE;

extern Contactor_t AIR_P;
extern Contactor_t AIR_N;
extern Contactor_t PRECHARGE;
extern Contactor_t DISCHARGE;

bool FSM_Transition(FSM_State_t* current_state, const FSM_State_t* new_state);

static inline bool FSM_TransitionToState(FSM_State_t* current_state, const FSM_State_t* new_state) {
    return FSM_Transition(current_state, new_state);
}


#endif // !FSM_H
