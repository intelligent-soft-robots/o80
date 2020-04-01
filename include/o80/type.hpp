// Copyright (c) 2019 Max Planck Gesellschaft
// Author : Vincent Berenz

#pragma once

#include "internal/time_stamp.hpp"

namespace o80
{
// duration : will try to reach target pressure over the given duration
// speed : will try to reach target pressure using the specified speed
// direct : just set the pressure to the fpga
enum Type
{
    DURATION,
    SPEED,
    DIRECT,
    DELAYED,
    ITERATION
};

class Speed
{
public:
    Speed()
    {
    }
    Speed(double _value) : value(_value)
    {
    }
    double value;
    template <class Archive>
    void serialize(Archive &archive)
    {
        archive(value);
    }
};

class Duration_us
{
public:
    Duration_us()
    {
    }
    Duration_us(long int _value) : value(_value)
    {
    }
    long int value;
    template <class Archive>
    void serialize(Archive &archive)
    {
        archive(value);
    }
};

class Direct
{
    template <class Archive>
    void serialize(Archive &archive)
    {
        archive();
    }
};

class Delayed
{
public:
    Delayed() : stamp(-1)
    {
    }
    Delayed(long int value) : stamp(value)
    {
    }
    Delayed(const TimePoint &time_stamp) : stamp(time_stamp.count())
    {
    }
    template <class Archive>
    void serialize(Archive &archive)
    {
        archive(stamp);
    }
    long int stamp;
};

class Iteration
{
public:
    Iteration()
    {
    }
    Iteration(long int _value) : value(_value)
    {
    }
    template <class Archive>
    void serialize(Archive &archive)
    {
        archive(value);
    }
    long int value;
};
}
