// Copyright (C) 2019 Max Planck Gesellschaft
// Author : Vincent Berenz

#pragma once

#include "o80/mode.hpp"
#include "o80/type.hpp"
#include "time_stamp.hpp"

namespace o80
{
class CommandType
{
public:
    CommandType(Speed speed);
    CommandType(Duration_us duration);
    CommandType(Iteration iteration);
    CommandType();
  
    // bool direct_passed() const;

    template <class Archive>
    void serialize(Archive &archive)
    {
        archive(type, speed, duration, iteration);
    }

public:
    Type type;
    Speed speed;
    Duration_us duration;
    Iteration iteration;
};
}
