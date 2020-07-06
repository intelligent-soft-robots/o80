#include "o80/bool_state.hpp"

namespace o80
{
BoolState::BoolState() : status_(false)
{
}

BoolState::BoolState(bool status) : status_(status)
{
}

void BoolState::set(bool value)
{
    status_ = value;
}

bool BoolState::get() const
{
    return status_;
}

std::string BoolState::to_string() const
{
    return std::to_string(status_);
}

bool BoolState::finished(const o80::TimePoint &start,
                         const o80::TimePoint &now,
                         const long int duration_us) const
{
    throw std::runtime_error("o80::BoolState duration command not supported");
}

bool BoolState::finished(const o80::TimePoint &start,
                         const o80::TimePoint &now,
                         const BoolState &start_state,
                         const BoolState &current_state,
                         const BoolState &previous_desired_state,
                         const BoolState &target_state,
                         const o80::Speed &speed) const
{
    throw std::runtime_error("o80::BoolState speed command not supported");
}

BoolState BoolState::intermediate_state(const o80::TimePoint &start,
                                        const o80::TimePoint &now,
                                        const BoolState &start_state,
                                        const BoolState &current_state,
                                        const BoolState &previous_desired_state,
                                        const BoolState &target_state,
                                        const o80::Speed &speed) const
{
    throw std::runtime_error("o80::BoolState speed command not supported");
}

BoolState BoolState::intermediate_state(
    const o80::TimePoint &start,
    const o80::TimePoint &now,
    const BoolState &start_state,
    const BoolState &current_state,
    const BoolState &previously_desired_state,
    const BoolState &target_state,
    const o80::Duration_us &duration) const
{
    throw std::runtime_error("o80::BoolState duration command not supported");
}

BoolState BoolState::intermediate_state(long int iteration_start,
                                        long int iteration_now,
                                        const BoolState &start_state,
                                        const BoolState &current_state,
                                        const BoolState &previous_desired_state,
                                        const BoolState &target_state,
                                        const o80::Iteration &iteration) const
{
    if (iteration_now == iteration.value)
    {
        return BoolState(true);
    }
    return BoolState(false);
}
}
