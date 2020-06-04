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
    static Speed per_second(double value)
    {
        return Speed(value / 1e6);
    }
    static Speed per_microsecond(double value)
    {
        return Speed(value);
    }
    static Speed per_millisecond(double value)
    {
        return Speed(value / 1e3);
    }
    static Speed per_nanosecond(double value)
    {
        return Speed(value * 1e3);
    }

public:
    double value;  // units per microsecond
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

    static Duration_us seconds(long int value)
    {
        Seconds s(value);
        Microseconds ms = std::chrono::duration_cast<Microseconds>(s);
        return Duration_us(ms.count());
    }

    static Duration_us milliseconds(long int value)
    {
        Milliseconds s(value);
        Microseconds ms = std::chrono::duration_cast<Microseconds>(s);
        return Duration_us(ms.count());
    }

    static Duration_us microseconds(long int value)
    {
        return Duration_us(value);
    }

    static Duration_us nanoseconds(long int value)
    {
        Nanoseconds s(value);
        Microseconds ms = std::chrono::duration_cast<Microseconds>(s);
        return Duration_us(ms.count());
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

class Iteration
{
public:
    Iteration() : value(-1), relative(false), do_reset(false)
    {
    }
    Iteration(long int _value) : value(_value), relative(false), do_reset(false)
    {
    }
    Iteration(long int _value, bool _relative)
        : value(_value), relative(_relative), do_reset(false)
    {
    }
    Iteration(long int _value, bool _relative, bool _do_reset)
        : value(_value), relative(_relative), do_reset(_do_reset)
    {
    }
    void reset()
    {
        do_reset = true;
    }
    template <class Archive>
    void serialize(Archive &archive)
    {
        archive(do_reset, relative, value);
    }
    long int value;
    bool relative;
    bool do_reset;
};
}
