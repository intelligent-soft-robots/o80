#pragma once

#include <chrono>
#include <eigen3/Eigen/Core>

namespace o80
{
typedef std::chrono::microseconds Microseconds;
typedef std::chrono::nanoseconds Nanoseconds;
typedef std::chrono::microseconds TimePoint;
typedef std::chrono::high_resolution_clock Clock;

/**
 * @return current time with microseconds precision
 */
TimePoint time_now();

/**
 * @return time difference in microseconds
 */
long int time_diff(const TimePoint& before, const TimePoint& after);
}
