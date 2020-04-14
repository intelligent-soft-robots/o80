// Copyright (C) 2019 Max Planck Gesellschaft
// Author : Vincent Berenz

#include "o80/internal/time_stamp.hpp"

namespace o80
{
TimeStamp::TimeStamp(long int stamp) : stamp_(stamp)
{
    if (stamp <= 0)
    {
        immediate_ = true;
    }
    else
    {
        immediate_ = false;
    }
}

TimeStamp::TimeStamp(Microseconds stamp) : stamp_(stamp)
{
    if (stamp.count() <= 0)
    {
        immediate_ = true;
    }
    else
    {
        immediate_ = false;
    }
}

TimeStamp::TimeStamp(const TimeStamp& stamp) : stamp_(stamp.stamp_)
{
    if (stamp.stamp_.count() <= 0)
    {
        immediate_ = true;
    }
    else
    {
        immediate_ = false;
    }
}

bool TimeStamp::passed() const
{
    if (immediate_) return true;

    TimePoint now = time_now();

    auto diff = stamp_ - now;

    if (diff.count() <= 0) return true;

    return false;
}

Microseconds TimeStamp::get_stamp() const
{
    return stamp_;
}
}
