#ifndef FSTUC_ERRNO_H
#define FSTUC_ERRNO_H

#if defined(__GNUC__) || defined(__clang__)
#include_next <errno.h>
#endif

#include <stdint.h>
#include <stdbool.h>

#define ERROR_INVALID_POINTER 0
#define ERROR_OUT_OF_BOUNDS 1
#define ERROR_TIMEOUT 2
#define ERROR_PERIPHERAL_FAILURE 3
#define ERROR_COMMUNICATION_FAILURE 4
#define ERROR_SENSOR_FAILURE 5
#define ERROR_MEMORY_FAILURE 7

#define MAX_ERRORS 32
#define MAX_PERIPHERALS 16

typedef struct {
    char peripheralName[64];
    uint32_t errorReg;
} ErrorRegister_t;

extern ErrorRegister_t PROGRAM_ERRORS;

static inline void Error_Set(ErrorRegister_t* reg, const uint8_t bit){
    if (reg && bit < MAX_ERRORS) {
        reg->errorReg |= (1U << bit);
    }
}

static inline void Error_Clear(ErrorRegister_t* reg, const uint8_t bit){
    if (reg && bit < MAX_ERRORS) {
        reg->errorReg &= ~(1U << bit);
    }
}

static inline bool Error_IsSet(const ErrorRegister_t* reg,const uint8_t bit){
    if (reg && bit < MAX_ERRORS) {
        return (reg->errorReg & (1U << bit)) != 0;
    }
    return false;
}

static inline bool Error_Match(const ErrorRegister_t* reg, const uint32_t faultMask) {
    if (reg) {
        return (reg->errorReg & faultMask) != 0;
    }
    return false;
}

static inline bool Error_AnySet(const ErrorRegister_t* reg) {
    if (reg) {
        return reg->errorReg != 0;
    }
    return false;
}

static inline void Error_ClearRegister(ErrorRegister_t* reg){
    if (reg) {
        reg->errorReg = 0;
    }
}

#endif // FSTUC_ERRNO_H

