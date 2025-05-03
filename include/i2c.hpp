#pragma once

#include "types.hpp"
#include "pin.hpp"

namespace eventide
{

class i2c
{
public:

    i2c()
    {}

    i2c(pin sda, pin scl, u8 address)
    {
        init(sda, scl, address);
    }

    ~i2c()
    {}

    void init(pin sda, pin scl, u8 address)
    {

    }

    void write(const void* data, size size)
    {}

    void read(void* data, size size) const
    {}

};
    
} // namespace eventide
