#include "o80/typedefs.hpp"
#include <iostream>

namespace o80
{
TimePoint time_now()
{
    std::chrono::steady_clock::time_point now = Clock::now();
    Nanoseconds m =
        std::chrono::duration_cast<Nanoseconds>(now.time_since_epoch());
    return m;
}

long int time_diff(const TimePoint& before, const TimePoint& after)
{
    return (after - before).count();
}

long int time_diff_us(const TimePoint& before, const TimePoint& after)
{
    return std::chrono::duration_cast<Microseconds>(after - before).count();
}
}
