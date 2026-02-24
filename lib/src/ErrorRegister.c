#include "ErrorRegister.h"

/**
 * @brief Global PROGRAM_ERRORS register instance
 */
ErrorRegister_t PROGRAM_ERRORS = {
    .peripheralName = "PROGRAM",
    .errorReg = 0
};
