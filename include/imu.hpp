#pragma once

#include "i2c.hpp"

namespace eventide
{

class imu : public i2c
{
    using base = i2c;

public:

    imu()
    {}

    imu(pin sda, pin scl)
    :   base(sda, scl, 0x68)
    {}
};
    
} // namespace eventide
