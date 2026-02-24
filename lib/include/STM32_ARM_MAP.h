#ifndef STM32_ARM_MAP_H
#define STM32_ARM_MAP_H

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


#endif // STM32_ARM_MAP_H