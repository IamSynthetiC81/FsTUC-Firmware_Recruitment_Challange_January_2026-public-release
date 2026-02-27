# Candidate Requirements

Welcome! This document outlines what you must do to complete the FsTUC Firmware Recruitment Challenge. Please read carefully and follow all instructions.

---

## 1. Technical Tasks

### 1.1. Required Implementations
- Implement the following modules only:
	- contactor.c
	- BatteryPack.c
	- FSM.c
- Ensure your code integrates with the existing C codebase and CMake build system.

### 1.2. Testing
- Some tests are already included in the tests directory. You are free to add more if you wish but do not remove or modify existing tests.
- Run all provided tests and ensure no failures before submission.

### 1.3. Code Quality & Safety
- Follow the project's coding style and conventions (see codebase for examples).
- Prioritize safety and reliability at all times. This is a High Voltage Controller—robustness is critical.
- Ensure your code is clear, maintainable, and efficient.

---

## 2. Architecture Overview

### 2.1. Physical Architecture
- **Contactor System:**
	- The HV controller manages multiple contactors (relays) to safely connect/disconnect the battery pack from the load.
	- Typical arrangement: Precharge contactor, Main contactors (AIR_P & AIR_N), Discharge contactor.
	- Each contactor is driven by a dedicated output and monitored for feedback.
	- See the PDF schematic in `doc/` for wiring and signal details. **Required reading.**

- **BatteryPack:**
	- Represents the HV battery pack, with its contactors and measurement interfaces.
	- Responsible for the safe managment of the isolating system, to ARM and DISARM the battery.

### 2.2. Program Architecture
- **Contactor Module (`contactor.c`):**
	- Implements control logic for each contactor.
	- Handles actuation, feedback monitoring, and error handling.
	- Provides APIs for FSM and BatteryPack to request state changes.

- **BatteryPack Module (`BatteryPack.c`):**
	- Manages battery measurements, health checks, and interface to contactors.
	- Reports faults and status to FSM.

- **FSM Module (`FSM.c`):**
	- Implements the main state machine for HV controller operation.
	- Coordinates transitions between safe, precharge, ready, discharge, and fault states.

---

## 3. FSM (Finite State Machine)

### 3.1. Required States
- **SAFE:**
	- All contactors open -> all current paths are broken. No current flow. System is in a safe, idle state.
- **PRECHARGE:**
	- Precharge current path closed, main and discharge current path open. Precharge resistor limits inrush current.
- **READY TO RACE:**
	- Main current path closed, precharge and discharge current paths open. System is ready for operation.
- **DISCHARGE:**
	- Discharge current path closed to safely discharge HV bus.
- **FAULT:**
	- Any detected fault. All contactors open, system locked out.

### 3.2. State Actions
- Each state must control contactors and battery pack as described above.
- Fault detection must trigger transition to FAULT state.

### 3.3. Transition Logic
- You must implement the transition logic in `FSM.c`.
- Transitions must be robust and **prioritize safety**.
- The above states MUST be implemented. You may add additional states if needed.

---

## 4. Process Requirements

- Documentation is already provided. No additional documentation is required.
- If you have any questions about the requirements, ask for clarification.
- Changelog updates are not needed for this challenge.

---

## 5. General Guidelines

- Ask for clarification if any requirement is unclear.
- Adhere to best practices for embedded systems programming.
- Respect deadlines and submission instructions.

---

## 6. Checklist

- [ ] contactor.c implemented
- [ ] BatteryPack.c implemented
- [ ] FSM.c implemented (with required states and transitions)
- [ ] All provided tests passing
- [ ] Code follows style and prioritizes safety/reliability
- [ ] Code builds without errors
- [ ] Submission steps completed
- [ ] PDF schematic in doc/ reviewed

---

Good luck!