#pragma once

#include <time.h>
#include <chrono>
#include "o80/time.hpp"

namespace o80
{
/*! class for imposing a frequency to a process*/
class FrequencyManager
{
public:
    /*! @param frequency: frequency to impose*/
    FrequencyManager(double frequency);
    /*! will sleep for the duration required so that the
     *  the period that passed since the last call
     *  matches the desired frequency */
    void wait();

private:
    timespec req_;
    TimePoint previous_time_;
    Nanoseconds period_;
};
}
