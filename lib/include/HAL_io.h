#ifndef HAL_IO_H
#define HAL_IO_H

#include <stdint.h>
#include <stdbool.h>

/* Clock Configuration */
#define CLOCK_CPU_HZ 48000000U

#define GPIO_PORT_COUNT 3
#define GPIO_PINS_PER_PORT 4

typedef enum {
    GPIO_PORT_A = 0,
    GPIO_PORT_B = 1,
    GPIO_PORT_C = 2
} GPIO_Port_t;

typedef enum {
    GPIO_PIN_0  = 0,
    GPIO_PIN_1  = 1,
    GPIO_PIN_2  = 2,
    GPIO_PIN_3  = 3
} GPIO_PinNumber_t;

typedef enum {
    GPIO_MODE_INPUT = 0,
    GPIO_MODE_OUTPUT = 1,
    GPIO_MODE_ALTERNATE = 2,
    GPIO_MODE_ANALOG = 3
} GPIO_Mode_t;

typedef enum {
    GPIO_PULL_NO = 0,
    GPIO_PULL_UP = 1,
    GPIO_PULL_DOWN = 2
} GPIO_Pull_t;

typedef struct GPIO_Pin_t {
    GPIO_Port_t port;
    GPIO_PinNumber_t pinNumber;
    GPIO_Mode_t mode;
    GPIO_Pull_t pull;
} GPIO_Pin_t;

/**
    * @brief Initializes a GPIO pin with specified configurations.
    *
    * @param pin Pointer to the GPIO_Pin_t structure defining the pin configuration.
    * @return true if initialization is successful, false otherwise.
    */
bool HAL_GPIO_Init(GPIO_Pin_t pin);

/**
    * @brief Writes a value to the specified GPIO pin.
    *
    * @param pin The GPIO pin to write to.
    * @param value The value to write (true for HIGH, false for LOW).
    * @return true if the operation is successful, false otherwise.
    */
bool HAL_GPIO_WritePin(GPIO_Pin_t pin, bool value);

/**
    * @brief Reads the value from the specified GPIO pin.
    *
    * @param pin The GPIO pin to read from.
    * @return The value read from the pin (true for HIGH, false for LOW).
    */
bool HAL_GPIO_ReadPin(GPIO_Pin_t pin);

/**
 * @brief Delays execution for a specified number of milliseconds.
 */
void HAL_Delay_ms(uint32_t milliseconds);

/**
 * @brief Delays execution for a specified number of microseconds.
 * 
 * @param microseconds Number of microseconds to delay
 */
void HAL_Delay_us(uint32_t microseconds);

/**
 * @brief Gets the current system tick count in CPU cycles.
 * @return Current cycle count since HAL initialization
 */
uint64_t HAL_GetTick(void);

double HAL_GetRuntimeMs(void);

/* Clock/Time Management Functions */

/**
 * @brief Initialize the HAL clock module.
 */
void HAL_Clock_Init(void);

/**
 * @brief Check if the clock module is initialized.
 */
bool HAL_Clock_IsInitialized(void);


#endif // HAL_IO_H