// Copyright (c) 2019 Max Planck Gesellschaft
// Author : Vincent Berenz

#pragma once

#include <array>

namespace o80
{
/**
 * ! States is a container of instances of NB_ACTUATORS
 *   instances of STATE. Expected usage is that STATE encapsulate
 *   the state of a specific actuator, hence States represents the
 *   full robot state.
 *   @tparam NB_ACTUATOR the number of actuators of the robot
 *   @tparam STATE class representing an actuator state
 */
template <int NB_ACTUATORS, class STATE>
class States
{
public:
    States()
    {
    }

    template <class Archive>
    void serialize(Archive& archive)
    {
        archive(values);
    }

    /**
     * ! set the state for the specified actuator
     */
    void set(int actuator, STATE state);

    /**
     * ! returns the state of the specified actuator
     */
    const STATE& get(int actuator) const;

    std::array<STATE, NB_ACTUATORS> values;
};

#include "states.hxx"
}  // namespace o80
