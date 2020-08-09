#pragma once

#include <chrono>
#include "o80/time.hpp"

namespace o80
{
/*! A class for evaluating the frequency
 *  at which a process run.
 */
class FrequencyMeasure
{
public:
    FrequencyMeasure();
    /*! @returns the frequency corresponding to the duration
        that passed since the previous call to tick*/
    double tick();

private:
    TimePoint previous_time_;
};
}  // namespace o80
