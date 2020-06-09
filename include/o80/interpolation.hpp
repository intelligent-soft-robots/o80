#pragma once

#include <iostream>
#include "o80/command_types.hpp"
#include "o80/time.hpp"

namespace o80
{
template <typename T>
/* ! returns true if the duration between
 * now and start is higher that the duration
 * required for interpolating between the target
 * and the start state at the provided speed.
 */
bool finished(const o80::TimePoint &start,
              const o80::TimePoint &now,
              const T &start_state,
              const T &current_state,
              const T &target_state,
              const o80::Speed &speed);

template <typename T>
/*! Interpolate between start and target state
 *  so that the state changes according to
 *  the provided speed.
 */
T intermediate_state(const o80::TimePoint &start,
                     const o80::TimePoint &now,
                     const T &start_state,
                     const T &current_state,
                     const T &target_state,
                     const o80::Speed &speed);

template <typename T>
/*! Interpolate between start and target state
 *  so that the target state is reached in the
 *  specified duration.
 */
T intermediate_state(const o80::TimePoint &start,
                     const o80::TimePoint &now,
                     const T &start_state,
                     const T &current_state,
                     const T &target_state,
                     const o80::Duration_us &duration);

template <typename T>
/*! Interpolate between start and target state
 *  so that the target state is reached at the
 *  specified iteration.
 */
T intermediate_state(long int start_iteration,
                     long int current_iteration,
                     const T &start_state,
                     const T &current_state,
                     const T &target_state,
                     const o80::Iteration &iteration);

#include "interpolation.hxx"
}
