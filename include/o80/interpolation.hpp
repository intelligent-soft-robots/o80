#pragma once

#include <iostream>
#include "o80/type.hpp"
#include "o80/typedefs.hpp"

namespace o80
{
bool finished(const o80::TimePoint &start,
              const o80::TimePoint &now,
              long int duration_us);

template <typename T>
bool finished(const o80::TimePoint &start,
              const o80::TimePoint &now,
              const T &start_state,
              const T &current_state,
              const T &target_state,
              const o80::Speed &speed);

template <typename T>
T intermediate_state(const o80::TimePoint &start,
                     const o80::TimePoint &now,
                     const T &start_state,
                     const T &current_state,
                     const T &target_state,
                     const o80::Speed &speed);

template <typename T>
T intermediate_state(const o80::TimePoint &start,
                     const o80::TimePoint &now,
                     const T &start_state,
                     const T &current_state,
                     const T &target_state,
                     const o80::Duration_us &duration);

template <typename T>
T intermediate_state(long int start_iteration,
                     long int current_iteration,
                     const T &start_state,
                     const T &current_state,
                     const T &target_state,
                     const o80::Iteration &iteration);

#include "interpolation.hxx"
}
