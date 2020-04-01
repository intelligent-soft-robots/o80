// Copyright (C) 2019 Max Planck Gesellschaft
// Author : Vincent Berenz

#pragma once

#include <chrono>
#include "o80/typedefs.hpp"

namespace o80
{
class TimeStamp
{
public:
    TimeStamp(long int stamp);
    TimeStamp(Microseconds stamp);
    TimeStamp(const TimeStamp& time_stamp);

    bool passed() const;
    Microseconds get_stamp() const;

private:
    Microseconds stamp_;
    bool immediate_;
};
}
