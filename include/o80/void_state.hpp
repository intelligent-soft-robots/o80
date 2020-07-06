#pragma once

#include "o80/command_types.hpp"
#include "o80/time.hpp"

namespace o80
{
class VoidState
{
public:
    VoidState();

    std::string to_string() const;

    void get() const
    {
    }
    void set() const
    {
    }

    bool finished(const o80::TimePoint &start,
                  const o80::TimePoint &now,
                  const long int duration_us) const;

    bool finished(const o80::TimePoint &start,
                  const o80::TimePoint &now,
                  const VoidState &start_state,
                  const VoidState &current_state,
                  const VoidState &previous_desired_state,
                  const VoidState &target_state,
                  const o80::Speed &speed) const;

    VoidState intermediate_state(const o80::TimePoint &start,
                                 const o80::TimePoint &now,
                                 const VoidState &start_state,
                                 const VoidState &current_state,
                                 const VoidState &previous_desired_state,
                                 const VoidState &target_state,
                                 const o80::Speed &speed) const;

    VoidState intermediate_state(const o80::TimePoint &start,
                                 const o80::TimePoint &now,
                                 const VoidState &start_state,
                                 const VoidState &current_state,
                                 const VoidState &previously_desired_state,
                                 const VoidState &target_state,
                                 const o80::Duration_us &duration) const;

    VoidState intermediate_state(long int iteration_start,
                                 long int iteration_now,
                                 const VoidState &start_state,
                                 const VoidState &current_state,
                                 const VoidState &previous_desired_state,
                                 const VoidState &target_state,
                                 const o80::Iteration &iteration) const;

    template <class Archive>
    void serialize(Archive &archive)
    {
    }
};
}
