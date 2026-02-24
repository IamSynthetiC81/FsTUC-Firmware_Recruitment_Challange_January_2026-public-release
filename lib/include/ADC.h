#ifndef ADC_H
#define ADC_H

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

#include "ErrorRegister.h"


/* =======================================================================
 * Battery Pack Error Bit and Structure Definitions
 * ====================================================================== */
#include "ErrorRegister.h"

#define FAULT_ADC_CONVERSION_ERROR_BIT 0
#define FAULT_ADC_BUFFER_OVERFLOW_BIT 1
#define FAULT_ADC_INVALID_CHANNEL_BIT 2
#define FAULT_ADC_UNSUPPORTED_FUNCTION_BIT 3

#define FAULT_ADC_HARD_MASK 0xFFFFFFFF

extern ErrorRegister_t ADCErrorRegister;

/*=========================================================================
 * ADC Definitions
 * ========================================================================== */
#define ADC_CLOCK                   480000 // 48 MHz

 #define ADC_MODE_POLLING 0
 #define ADC_MODE_AUTOMATIC 1

 #define ADC_RESOLUTION_8BIT 0x00
 #define ADC_RESOLUTION_10BIT 0x01
 #define ADC_RESOLUTION_12BIT 0x02
 #define ADC_RESOLUTION_14BIT 0x03

 #define ADC_CHANNEL_0 0x01
 #define ADC_CHANNEL_1 0x02
 #define ADC_CHANNEL_2 0x04
 #define ADC_MAX_CHANNELS            3

 #define ADC_PRESCALER_DIV1 0x00
 #define ADC_PRESCALER_DIV2 0x01
 #define ADC_PRESCALER_DIV4 0x02
 #define ADC_PRESCALER_DIV8 0x03
 #define ADC_PRESCALER_DIV16 0x04
 #define ADC_PRESCALER_DIV32 0x05
 #define ADC_PRESCALER_DIV64 0x06
 #define ADC_PRESCALER_DIV128 0x07

 #define ADC_MAX_VALUE ((1U << ADC_RESOLUTION_BITS) - 1)
 #define ADC_VOLTAGE_REF 3.3f

 #define ADC_BUFFER_SIZE             21
 #


/*=========================================================================
 * ADC Driver Interface
 * ========================================================================== */

typedef struct __attribute__((packed)) {
    unsigned int mode          : 1;     // 0 = Polling, 1 = Automatic
    unsigned int resolution    : 2;     // 00 = 8-bit, 01 = 10-bit, 10 = 12-bit, 11 = 14-bit
    unsigned int channelEnable : 3;     // Bitmask to enable channels 0-2
} ADC_Config_t;

typedef struct __attribute__((packed)) {
    unsigned int conversion_in_progress : 1;
    unsigned int currentChannel         : 2;
    unsigned int reserved               : 5;
} ADC_Status_t;

typedef struct __attribute__((packed)) {
    unsigned int prescaler         : 8;
    unsigned int circular_buffer   : 1;
    unsigned int bufferFullFlag    : 1;
    unsigned int bufferSampleCount : 5;
} ADC_AutomaticModeConfig_t;

typedef struct __attribute__((packed)) {
    unsigned int BufferOverflowError : 1;
    unsigned int ConversionError     : 1;
    unsigned int Reserved            : 6;
} ADC_ErrorRegister_t;

typedef struct {
    unsigned int raw_value : 14;
} ADC_Sample_t;

typedef struct {
    uint16_t samples[12];
} ADC_Frame_t;

typedef struct {
    ADC_Frame_t frame;
    size_t sample_count;
} ADC_FrameBuffer_t;

typedef struct {
    ADC_Config_t config;
    ADC_Status_t status;
    ADC_AutomaticModeConfig_t auto_config;
    ADC_FrameBuffer_t buffer;
    ADC_ErrorRegister_t error_reg;

    void (*sample_ready_callback)(const ADC_Sample_t *sample, void *ctx);
    void (*frame_complete_callback)(const ADC_Frame_t *frame, void *ctx);

    void *sample_callback_ctx;
    void *frame_callback_ctx;
} ADC_Handle_t;

bool ADC_Init(ADC_Handle_t *adc, const ADC_Config_t *cfg);
bool ADC_Start(ADC_Handle_t *adc);
bool ADC_Stop(ADC_Handle_t *adc);
bool ADC_GetSample(ADC_Handle_t *adc, ADC_Sample_t *sample);
uint16_t ADC_Read(ADC_Handle_t *adc,const uint8_t channel);
bool ADC_RegisterSampleCallback(ADC_Handle_t *adc,
                                void (*callback)(const ADC_Sample_t *, void *),
                                void *ctx);
bool ADC_RegisterFrameCallback(ADC_Handle_t *adc,
                               void (*callback)(const ADC_Frame_t *, void *),
                               void *ctx);

#endif // ADC_H
