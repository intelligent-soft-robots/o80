#pragma once

#include <string>
#include <tuple>
#include "o80/interpolation.hpp"

namespace o80
{
/*! A State represents the state of an actuator,
 *  as well as methods specifying how a state
 *  interpolates toward another. The interpolation
 *  (and finished) method will be used by the BackEnd
 *  to compute at each iteration the current desired
 *  state value for each actuator, based on commands
 *  sent by FrontEnd. E.g. if a command requests the desired
 *  state of an actuator to reach value "target" in 5 seconds,
 *  considering the current state of the actuator is "start",
 *  then the interpolation methods will specified how the
 *  desired state value interpolates between "start" and "target".
 *  By default linear interpolation methods are implemented, but
 *  they may be overriden for better control. The default linear
 *  interpolation will work only if the state value is of native type
 *  (e.g. double, float) and has to be overriden for any other type.
 *  @tparam T value of the state
 */
template <class Sub, typename... Args>
class MdState
{
public:
    State(Args &&... args);
    State();

    template <int INDEX>
    typename std::tuple_element<INDEX, std::tuple<Args...> >::type get()
    {
        return std::get<INDEX>(values_);
    }

    template <int INDEX>
    void set(
        typename std::tuple_element<INDEX, std::tuple<Args...> >::type value)
    {
        std::get<INDEX>(values_) = value;
    }

    template <int INDEX>
    T get() const;
    void set(T value);
    std::string to_string() const;

    bool finished(const o80::TimePoint &start,
                  const o80::TimePoint &now,
                  const Sub &start_state,
                  const Sub &current_state,
                  const Sub &previous_desired_state,
                  const Sub &target_state,
                  const o80::Speed &speed) const;

    Sub intermediate_state(const o80::TimePoint &start,
                           const o80::TimePoint &now,
                           const Sub &start_state,
                           const Sub &current_state,
                           const Sub &previous_desired_state,
                           const Sub &target_state,
                           const o80::Speed &speed) const;

    Sub intermediate_state(const o80::TimePoint &start,
                           const o80::TimePoint &now,
                           const Sub &start_state,
                           const Sub &current_state,
                           const Sub &previous_desired_state,
                           const Sub &target_state,
                           const o80::Duration_us &duration) const;

    Sub intermediate_state(long int start_iteration,
                           long int current_iteration,
                           const Sub &start_state,
                           const Sub &current_state,
                           const Sub &previous_desired_state,
                           const Sub &target_state,
                           const o80::Iteration &iteration) const;

    template <class Archive>
    void serialize(Archive &archive)
    {
        archive(value);
    }

    std::tuple<Args...> values_;
};

#include "state.hxx"

}  // namespace o80
