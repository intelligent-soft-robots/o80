#pragma once

namespace o80
{
/**
 *  @brief considering a starting iteration, a target iteration,
 *  at starting state and a target state, compute the desired
 *  state for the current iteration using simple linear interpolation.
 */
double linear_interpolation(long int iteration_start,
                            long int iteration_now,
                            double start_state,
                            double current,
                            double target_state,
                            long int iteration);

/**
 *  @brief considering a starting iteration, a target iteration,
 *  at starting state and a target state, compute the desired
 *  state for the current iteration using simple linear interpolation.
 */
int linear_interpolation(long int iteration_start,
                         long int iteration_now,
                         int start_state,
                         int current,
                         int target_state,
                         long int iteration);

bool finished(const o80::TimePoint &start,
              const o80::TimePoint &now,
              double starting,
              double current,
              double target,
              const o80::Speed &speed);

bool finished(const o80::TimePoint &start,
              const o80::TimePoint &now,
              int starting,
              int current,
              int target,
              const o80::Speed &speed);
}
