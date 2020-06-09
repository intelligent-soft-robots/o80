// Copyright (c) 2019 Max Planck Gesellschaft
// Author : Vincent Berenz

#pragma once

#include "internal/time_stamp.hpp"

namespace o80
{

  /**
   * ! commands type, used by the add_commands methods of FrontEnd.
   * -  duration : will try to reach target state over the given duration
   * - speed : will try to reach target state using the specified speed
   * - direct : will request to set the target state direcly
   * - iteration: will request for the target state to be reached at the
   *              provided backend iteration.
   */
enum Type
{
    DURATION,
    SPEED,
    DIRECT,
    ITERATION
};

  /**
   * ! for interpolating toward the desired state using
   *   a specified velocity.
   */
class Speed
{
public:
    Speed()
    {
    }
  /* ! in unit of state per microseconds */
    Speed(double _value) : value(_value)
    {
    }
  /* ! construct a speed based on unit per state per second*/ 
    static Speed per_second(double value)
    {
        return Speed(value / 1e6);
    }
  /* ! construct a speed based on unit per state per microsecond*/ 
    static Speed per_microsecond(double value)
    {
        return Speed(value);
    }
    /* ! construct a speed based on unit per state per millisecond*/ 
    static Speed per_millisecond(double value)
    {
        return Speed(value / 1e3);
    }
    /* ! construct a speed based on unit per state per nanosecond*/ 
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

  /**
   * ! for interpolating toward the desired state 
   *   during a specified duration
   */
  class Duration_us
{
public:
    Duration_us()
    {
    }
  /* ! value in microseconds */
    Duration_us(long int _value) : value(_value)
    {
    }

  /* ! construct a duration from seconds */
    static Duration_us seconds(long int value)
    {
        Seconds s(value);
        Microseconds ms = std::chrono::duration_cast<Microseconds>(s);
        return Duration_us(ms.count());
    }

  /* ! construct a duration from milliseconds */
    static Duration_us milliseconds(long int value)
    {
        Milliseconds s(value);
        Microseconds ms = std::chrono::duration_cast<Microseconds>(s);
        return Duration_us(ms.count());
    }

    /* ! construct a duration from microseconds */
    static Duration_us microseconds(long int value)
    {
        return Duration_us(value);
    }

  /* ! construct a duration from nanoseconds */
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

  /* ! To have the desired state applied directly*/
class Direct
{
    template <class Archive>
    void serialize(Archive &archive)
    {
        archive();
    }
};

  /**
   * ! for interpolating toward the desired state 
   *   such at reaching it a the specified iteration
   */

  class Iteration
{
public:
    Iteration() : value(-1), relative(false), do_reset(false)
    {
    }
    Iteration(long int iteration) : value(iteration), relative(false), do_reset(false)
    {
    }
  /* ! if relative is true, then the iteraton
  *    is not the BackEnd iteration number as counted since the Backend started,
  *    but the iteration since the last command requesting an iteration reset
  *    has been started (see Iteration(long int, bool, bool))
  */
    Iteration(long int iteration, bool _relative)
        : value(iteration), relative(_relative), do_reset(false)
    {
    }
  /* ! if reset true, reset the relative iteration count to zero
   *
   */
    Iteration(long int iteration, bool _relative, bool _do_reset)
        : value(iteration), relative(_relative), do_reset(_do_reset)
    {
    }
  /* ! the command encapsulating this iteration instance will request 
   *   the BackEnd to reset the relative iteration number to 0
   */
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
