#include "o80/typedefs.hpp"
#include <iostream>

namespace o80
{
TimePoint time_now()
{
    std::chrono::system_clock::time_point now = Clock::now();
    Microseconds m =
        std::chrono::duration_cast<Microseconds>(now.time_since_epoch());
    return m;
}

long int time_diff(const TimePoint& before, const TimePoint& after)
{
    return (after - before).count();
}
}
