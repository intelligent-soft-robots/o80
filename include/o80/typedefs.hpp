#pragma once

#include <chrono>
#include <eigen3/Eigen/Core>

namespace o80
{
typedef std::chrono::microseconds Microseconds;
typedef std::chrono::nanoseconds Nanoseconds;
typedef std::chrono::nanoseconds TimePoint;
typedef std::chrono::steady_clock Clock;

    
/**
 * @return current time with nanoseconds precision
 */
TimePoint time_now();

/**
 * @return time difference in nanoseconds
 */
long int time_diff(const TimePoint& before, const TimePoint& after);

}
