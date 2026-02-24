/**
 * @file ADC.c
 * @brief ADC Peripheral Driver Implementation
 * 
 * Microcontroller-style ADC driver supporting:
 * - 14-bit resolution
 * - 3 channels (1-3)
 */



#include "ADC.h"
#include "CircuitSim.h"

#include <stdio.h>
#include <string.h>

ErrorRegister_t ADCErrorRegister = {
    .peripheralName = "ADC",
    .errorReg = 0
};

static uint16_t ADC_MAX_RAW_VALUE;
#define ADC_VOLTAGE_MAX_V 600.0
#define ADC_CURRENT_MAX_A 200.0

static bool ADC_ValidateConfig(const ADC_Config_t *cfg) {
    return (cfg != NULL);
}

static double _clamp_double(const double value,const double min_value,const double max_value) {
    if (value < min_value) {
        return min_value;
    }
    if (value > max_value) {
        return max_value;
    }
    return value;
}


static uint16_t _scale_voltage_to_raw(const double voltage_v) {
    const double clamped = _clamp_double(voltage_v, 0.0, ADC_VOLTAGE_MAX_V);
    const double normalized = clamped / ADC_VOLTAGE_MAX_V;


    return (uint16_t)((normalized * (double)ADC_MAX_RAW_VALUE) + 0.5);
}

static uint16_t _scale_current_to_raw(const double current_a) {
    const double clamped = _clamp_double(current_a, -ADC_CURRENT_MAX_A, ADC_CURRENT_MAX_A);
    const double normalized = (clamped + ADC_CURRENT_MAX_A) / (2.0 * ADC_CURRENT_MAX_A);

    return (uint16_t)((normalized * (double)ADC_MAX_RAW_VALUE) + 0.5);
}




static uint8_t _next_enabled_channel(const uint8_t current_channel, const uint8_t channel_mask) {
    uint8_t channel = current_channel;

    for (uint8_t i = 0U; i < ADC_MAX_CHANNELS; i++) {
        channel++;
        if (channel > ADC_MAX_CHANNELS) {
            channel = 1U;
        }

        if (channel_mask & (uint8_t)(1U << (channel - 1U))) {
            return channel;
        }
    }

    return current_channel;
}



/* ============================================================================
 * Public ADC API Implementation
 * ========================================================================== */

bool ADC_Init(ADC_Handle_t *adc, const ADC_Config_t *cfg) {
    if (!adc || !ADC_ValidateConfig(cfg)) {
        return false;
    }

    if (cfg->mode == ADC_MODE_AUTOMATIC) {
        Error_Set(&ADCErrorRegister, FAULT_ADC_UNSUPPORTED_FUNCTION_BIT);
        fprintf(stderr, "ADC_Init: Automatic mode is currently unsupported\n");
        return false;
    }

    adc->config = *cfg;
    /* Initialize ADC state */
    adc->status.conversion_in_progress = 0;
    adc->status.currentChannel = 1; // Default to channel 1 (1-3)
    if (cfg->channelEnable & 0x02) {
        adc->status.currentChannel = 2;
    }
    if (cfg->channelEnable & 0x04) {
        adc->status.currentChannel = 3;
    }

    /* Initialize Automatic Mode Configuration */
    adc->auto_config.prescaler = 1;
    adc->auto_config.circular_buffer = 0;
    adc->auto_config.bufferFullFlag = 0;
    adc->auto_config.bufferSampleCount = 0; 

    /* Initialize buffer */
    memset(adc->buffer.frame.samples, 0, sizeof(adc->buffer.frame.samples));
    adc->buffer.sample_count = 0;

    /* Resolution encoding: 0=8-bit, 1=10-bit, 2=12-bit, 3=14-bit.
     * Bit width = 8 + 2*resolution, so max raw value = (1 << (8 + 2*resolution)) - 1. */
    ADC_MAX_RAW_VALUE = (1U << (8U + 2U * (uint8_t)cfg->resolution)) - 1U;

    return true;
}

/**
 * Start ADC conversion based on automatic mode configuration. 
 * In automatic mode, ADC continuously samples and fills the buffer by cycling though each enabled channel in order (0 -> 2).
 * If circular_buffer is enabled, it wraps around and continues filling, otherwise it stops until the buffer is processed.
 * The frame_complete_callback is called when a buffer is filled, and the sample_ready_callback is called for each sample as it is acquired.
 * 
 * In polling mode, this function is not used and ADC_GetSample() should be called directly to retrieve samples.
 */
bool ADC_Start(ADC_Handle_t *adc) {
    if (!adc) {
        return false;
    }

    if (adc->config.mode != ADC_MODE_AUTOMATIC) {
        return false;
    }

    adc->status.conversion_in_progress = 1;
    // Automatic mode currently unsuported !!!
    return true;
}

bool ADC_Stop(ADC_Handle_t *adc) {
    if (!adc) {
        return false;
    }

    if (adc->config.mode != ADC_MODE_AUTOMATIC) {
        return false;
    }

    adc->status.conversion_in_progress = 0;
    // Automatic mode currently unsuported !!!
    return true;
}

bool ADC_GetSample(ADC_Handle_t *adc, ADC_Sample_t *sample) {
    if (!adc || !sample) {
        return false;
    }

    if (adc->config.mode != ADC_MODE_POLLING) {
        return false;
    }

    /* In polling mode, get channel , and then get appropriate channel */
    {
        uint8_t channel_bit = (uint8_t)(1U << (adc->status.currentChannel - 1U));
        if ((adc->config.channelEnable & channel_bit) == 0) {
            adc->error_reg.ConversionError = 1;
            return false;  /* Channel not enabled */
        }
    }

    if (adc->status.currentChannel == 1U) {
        // Channel 1: Vehicle voltage (V_LOAD)
        sample->raw_value = _scale_voltage_to_raw(get_vehicle_voltage());
    } else if (adc->status.currentChannel == 2U) {
        // Channel 2: Voltage difference (battery minus vehicle)
        sample->raw_value = _scale_voltage_to_raw(get_battery_voltage() - get_vehicle_voltage());
    } else {
        // Channel 3: Battery discharge current
        sample->raw_value = _scale_current_to_raw(get_battery_current());
    }

    adc->status.currentChannel = _next_enabled_channel(adc->status.currentChannel,
                                                       adc->config.channelEnable);

    /* In other modes, samples are provided via callbacks and this function is not used */
    return true;
}

bool ADC_RegisterSampleCallback(ADC_Handle_t *adc, void (*callback)(const ADC_Sample_t *, void *),void *ctx) {
    if (!adc || !callback) {
        return false;
    }
    
    adc->sample_ready_callback = callback;
    adc->sample_callback_ctx = ctx;
    return true;
}

bool ADC_RegisterFrameCallback(ADC_Handle_t *adc, void (*callback)(const ADC_Frame_t *, void *),void *ctx) {
    if (!adc || !callback) {
        return false;
    }
    
    adc->frame_complete_callback = callback;
    adc->frame_callback_ctx = ctx;
    return true;
}

uint16_t ADC_Read(ADC_Handle_t *adc,const uint8_t channel) {
    if (!adc || channel == 0 || channel > ADC_MAX_CHANNELS) {
        return 0;  /* Invalid parameters */
    }

    uint8_t channel_bit = (uint8_t)(1U << (channel - 1U));
    if ((adc->config.channelEnable & channel_bit) == 0) {
        adc->error_reg.ConversionError = 1;
        return 0;  /* Channel not enabled */
    }

    if (channel == 1U) {
        return _scale_voltage_to_raw(get_battery_voltage());
    } else if (channel == 2U) {
        return _scale_voltage_to_raw(get_vehicle_voltage());
    } else {
        return _scale_current_to_raw(get_battery_current());
    }

}
