# FsTUC Firmware Recruitment Challenge (Participant Release)

This repository contains the participant-facing version of the challenge.

## What You Implement

You are evaluated primarily on:

- `lib/src/contactor.c`
- `lib/src/BatteryPack.c`
- `lib/src/FSM.c`

Header files are provided in `lib/include`.

---

## Hidden Embedded Libraries

The following embedded modules are provided as a prebuilt static library:

- `ADC`
- `EEPROM`
- `HAL_io`
- `CircuitSim`
- `Clock`
- `Errno`

Do not modify prebuilt artifacts.

---

## Requirements

### Common

- CMake >= 3.20
- C compiler (GCC/Clang/MSVC)
- Git
- Python 3 (for helper/report scripts)

### Linux (Ubuntu/Debian)

- `build-essential`
- `cmake`
- `ninja-build`
- `python3`
- `python3-pip`
- `git`

### macOS

- Homebrew
- `cmake`
- `ninja`
- `python`
- `git`

### Windows

- Git for Windows
- CMake
- Ninja
- Python 3.12+
- Visual Studio Build Tools (or Visual Studio with C++ tools)

---

## Installation

### Linux / macOS

```bash
bash setup.sh
```

### Windows (PowerShell)

```powershell
./setup.ps1
```

---

## Build

From repository root:

### On Linux/macOS with Unix Makefiles generator:
```bash
cmake -B build -G "Unix Makefiles"
cmake --build build
```

### On Windows with Ninja generator:
```powershell
cmake -B build -G "Ninja"
cmake --build build
```

> Note: You can also use Visual Studio generator on Windows if you prefer.

## Run Tests

```bash
ctest --test-dir build --output-on-failure
```

Or run individual test binaries from `build/`.

---

## Notes

- Keep APIs in the provided headers intact.
- Focus on correctness, safety checks, and clear state-machine behavior.
- Your implementation should be deterministic and testable.

Good luck!
