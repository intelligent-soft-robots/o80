// Copyright (c) 2019 Max Planck Gesellschaft
// Author : Vincent Berenz

#pragma once

#include "time_stamp.hpp"

namespace o80
{
/**
 * Types of commands
 */
enum Type
{
    SPEED,   ///< the command applies an interpolation to enforce a given speed
    DIRECT,  ///< the command desired state becomes the joint desired state ASAP
    ITERATION,  ///< the command desired state becomes the joint desired state
                /// at iteration
};

/**
 * To be used as parameter of a command, to indicate
 * the command will be of type Speed.
 */
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

/**
 * To be used as parameter of a command, to indicate
 * the command will be of type direct.
 * Value in microseconds.
 */
class Direct
{
    template <class Archive>
    void serialize(Archive &archive)
    {
        archive();
    }
};

/**
 * To be used as parameter of a command, to indicate
 * the command will be of type iteration.
 * Value in microseconds.
 */
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
