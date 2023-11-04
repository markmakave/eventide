#pragma once

#include <driver/adc.h>

namespace eventide
{
    
class adc {
public:

    adc(uint8_t pin) : _pin(pin) {
        adc1_config_width(ADC_WIDTH_BIT_12);
        adc1_config_channel_atten(_pin, ADC_ATTEN_DB_11);
    }

protected:

    uint8_t _pin;
};

} // namespace eventide
