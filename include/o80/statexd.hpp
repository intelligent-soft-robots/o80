#pragma once

#include <string>
#include <tuple>
#include <utility>
#include "o80/interpolation.hpp"

namespace o80
{
/*! Similarly to an instance of State,
 *  an instance of StateXd represents the state of an actuator,
 *  and provides methods for iterpolating between states.
 *  The difference with State is that StateXd supports
 *  several attributes, e.g.
 *  \code{.cpp}
 *  class Joint2d : public StateXd<Joint2d,int,double>
 *  \endcode
 *  Joint2d will encapsulate an int and a double attribute.
 */
template <class Sub, typename... Args>
class StateXd
{
public:
    StateXd(Args... args);
    StateXd();

    /*! returns the INDEXth attributes */
    template <int INDEX>
    typename std::tuple_element<INDEX, std::tuple<Args...> >::type get() const;

    /*! set the INDEXth attribute */
    template <int INDEX>
    void set(
        typename std::tuple_element<INDEX, std::tuple<Args...> >::type value);

    /*! returns true if the speed command finished for the attribute
     *  at the first (0) index
     */
    bool finished(const o80::TimePoint &start,
                  const o80::TimePoint &now,
                  const Sub &start_state,
                  const Sub &current_state,
                  const Sub &previous_desired_state,
                  const Sub &target_state,
                  const o80::Speed &speed) const;

    /* ! uses linear interpolation to compute the desired state
     *   at TimePoint "now" provided the speed command. Note that the speed
     *   of the command corresponds to the first (0) index attribute.
     *   The other attribute will interpolate using a duration command
     *   inferred from the speed command applied to the first index. */
    Sub intermediate_state(const o80::TimePoint &start,
                           const o80::TimePoint &now,
                           const Sub &start_state,
                           const Sub &current_state,
                           const Sub &previous_desired_state,
                           const Sub &target_state,
                           const o80::Speed &speed) const;

    /* ! uses linear interpolation to compute the desired state
     *   at TimePoint "now" provided the duration command. */
    Sub intermediate_state(const o80::TimePoint &start,
                           const o80::TimePoint &now,
                           const Sub &start_state,
                           const Sub &current_state,
                           const Sub &previous_desired_state,
                           const Sub &target_state,
                           const o80::Duration_us &duration) const;

    /* ! uses linear interpolation to compute the desired state
     *   at TimePoint "now" provided the iteration command. */
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
        std::apply(archive, values_);
    }

private:
    std::tuple<Args...> values_;
};

#include "statexd.hxx"

}  // namespace o80
