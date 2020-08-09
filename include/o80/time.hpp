#pragma once

#include <chrono>
#include <eigen3/Eigen/Core>

namespace o80
{
typedef std::chrono::seconds Seconds;
typedef std::chrono::milliseconds Milliseconds;
typedef std::chrono::microseconds Microseconds;
typedef std::chrono::nanoseconds Nanoseconds;
typedef std::chrono::nanoseconds TimePoint;
typedef std::chrono::steady_clock Clock;

/**
 * ! return current time with nanoseconds precision
 */
TimePoint time_now();

/**
 * ! returns the duration between after and before, in nanoseconds
 */
long int time_diff(const TimePoint& before, const TimePoint& after);

/**
 * ! returns the duration between after and before, in microseconds
 */
long int time_diff_us(const TimePoint& before, const TimePoint& after);
}  // namespace o80
