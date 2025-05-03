#pragma once

namespace eventide
{

class pin
{
public:

    explicit pin(int id)
    :   _id(id)
    {}

    operator int() const
    {
        return _id;
    }

protected:

    int _id;
};
    
} // namespace eventide
