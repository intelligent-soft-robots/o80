#include "o80/frequency_manager.hpp"
#include <iostream>

namespace o80
{
FrequencyManager::FrequencyManager(double frequency)
    : period_(static_cast<long int>((1e9 / frequency) + 0.5)),
      previous_time_(time_now())
{
}

void FrequencyManager::wait()
{
    TimePoint now = time_now();
    Nanoseconds time_diff = period_ - now + previous_time_;
    long int td = time_diff.count();
    if (td > 0)
    {
        req_.tv_sec = 0;
        req_.tv_nsec = td;
        nanosleep(&req_, NULL);
    }
    previous_time_ = now + time_diff;
}
}
