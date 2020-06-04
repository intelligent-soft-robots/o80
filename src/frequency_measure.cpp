#include "o80/frequency_measure.hpp"
#include <iostream>

namespace o80
{
FrequencyMeasure::FrequencyMeasure() : previous_time_(time_now())
{
}

double FrequencyMeasure::tick()
{
    TimePoint now = time_now();
    Nanoseconds time_diff = now - previous_time_;
    double frequency = 1e9 / (static_cast<double>(time_diff.count()));
    previous_time_ = now;
    return frequency;
}
}
