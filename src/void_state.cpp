#include "o80/void_state.hpp"

namespace o80
{
VoidState::VoidState()
{
}

std::string VoidState::to_string() const
{
    return std::string("");
}

bool VoidState::finished(const o80::TimePoint &start,
                         const o80::TimePoint &now,
                         const long int duration_us) const
{
    throw std::runtime_error("o80::VoidState duration command not supported");
}

bool VoidState::finished(const o80::TimePoint &start,
                         const o80::TimePoint &now,
                         const VoidState &start_state,
                         const VoidState &current_state,
                         const VoidState &previous_desired_state,
                         const VoidState &target_state,
                         const o80::Speed &speed) const
{
    throw std::runtime_error("o80::VoidState speed command not supported");
}

VoidState VoidState::intermediate_state(const o80::TimePoint &start,
                                        const o80::TimePoint &now,
                                        const VoidState &start_state,
                                        const VoidState &current_state,
                                        const VoidState &previous_desired_state,
                                        const VoidState &target_state,
                                        const o80::Speed &speed) const
{
    throw std::runtime_error("o80::VoidState speed command not supported");
}

VoidState VoidState::intermediate_state(
    const o80::TimePoint &start,
    const o80::TimePoint &now,
    const VoidState &start_state,
    const VoidState &current_state,
    const VoidState &previously_desired_state,
    const VoidState &target_state,
    const o80::Duration_us &duration) const
{
    throw std::runtime_error("o80::VoidState duration command not supported");
}

VoidState VoidState::intermediate_state(long int iteration_start,
                                        long int iteration_now,
                                        const VoidState &start_state,
                                        const VoidState &current_state,
                                        const VoidState &previous_desired_state,
                                        const VoidState &target_state,
                                        const o80::Iteration &iteration) const
{
    throw std::runtime_error("o80::VoidState iteration command not supported");
}
}
