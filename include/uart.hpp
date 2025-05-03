#pragma once

#include <stdexcept>
#include <utility>

#include <driver/uart.h>

#include "types.hpp"
#include "pin.hpp"

namespace eventide
{

class uart
{
    static constexpr size_t rx_buffer_size = 1024, tx_buffer_size = 1024;

public:

    uart()
    :   _port(UART_NUM_MAX)
    {}

    template <u8 port = 0>
    uart(pin rx, pin tx, int baudrate = 115200)
    :   _port(_port_cvt(port))
    {
        init(rx, tx, baudrate);
    }

    void init(pin rx, pin tx, int baudrate)
    {
        uart_config_t uart_config = {
            .baud_rate = baudrate,
            .data_bits = UART_DATA_8_BITS,
            .parity = UART_PARITY_DISABLE,
            .stop_bits = UART_STOP_BITS_1,
            .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
            .source_clk = UART_SCLK_DEFAULT,
        };

        ESP_ERROR_CHECK(uart_driver_install(_port, rx_buffer_size, tx_buffer_size, 0, NULL, 0));
        ESP_ERROR_CHECK(uart_param_config(_port, &uart_config));
        ESP_ERROR_CHECK(uart_set_pin(_port, tx, rx, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    }

    void write(const void* data, size_t size)
    {
        size_t sent = uart_write_bytes(_port, data, size);
        if (sent != size)
            throw std::runtime_error("uart write failed");
    }

    void read(void* data, size_t size, TickType_t timeout = portMAX_DELAY) const
    {
        size_t read = uart_read_bytes(_port, data, size, timeout);
        if (read != size)
            throw std::runtime_error("uart read failed");
    }

protected:

    static constexpr uart_word_length_t _data_bits_cvt(size_t x)
    {
        switch (x)
        {
            case 5: return UART_DATA_5_BITS;
            case 6: return UART_DATA_6_BITS;
            case 7: return UART_DATA_7_BITS;
            case 8: return UART_DATA_8_BITS;
        }
        std::unreachable();
    }

    static constexpr uart_port_t _port_cvt(u8 port)
    {
        switch (port)
        {
            case 0: return UART_NUM_0;
            case 1: return UART_NUM_1;
            case 2: return UART_NUM_2;
        }
        std::unreachable();
    }

protected:

    uart_port_t _port;
};
    
} // namespace eventide
