#pragma once

#include "o80/interpolation.hpp"

namespace o80
{
/*! A State encapsulating a boolean
 *
 */
class BoolState
{
public:
    BoolState();
    BoolState(bool value);

    void set(bool value);
    bool get() const;
    std::string to_string() const;

    bool finished(const o80::TimePoint &start,
                  const o80::TimePoint &now,
                  const long int duration_us) const;

    bool finished(const o80::TimePoint &start,
                  const o80::TimePoint &now,
                  const BoolState &start_state,
                  const BoolState &current_state,
                  const BoolState &previous_desired_state,
                  const BoolState &target_state,
                  const o80::Speed &speed) const;

    BoolState intermediate_state(const o80::TimePoint &start,
                                 const o80::TimePoint &now,
                                 const BoolState &start_state,
                                 const BoolState &current_state,
                                 const BoolState &previous_desired_state,
                                 const BoolState &target_state,
                                 const o80::Speed &speed) const;

    BoolState intermediate_state(const o80::TimePoint &start,
                                 const o80::TimePoint &now,
                                 const BoolState &start_state,
                                 const BoolState &current_state,
                                 const BoolState &previously_desired_state,
                                 const BoolState &target_state,
                                 const o80::Duration_us &duration) const;

    BoolState intermediate_state(long int iteration_start,
                                 long int iteration_now,
                                 const BoolState &start_state,
                                 const BoolState &current_state,
                                 const BoolState &previous_desired_state,
                                 const BoolState &target_state,
                                 const o80::Iteration &iteration) const;

    template <class Archive>
    void serialize(Archive &archive)
    {
        archive(status_);
    }

private:
    bool status_;
};
}
