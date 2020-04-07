// Copyright (c) 2019 Max Planck Gesellschaft
// Author : Vincent Berenz

#pragma once

#include <sstream>
#include "shared_memory/serializer.hpp"
#include "states.hpp"
#include "typedefs.hpp"

namespace o80

{
// Observation (below) as an optional template (Extended State).
// The class EmptyExtendedState is used if this templated is unspecified.
class EmptyExtendedState
{
public:
    EmptyExtendedState()
    {
    }
    EmptyExtendedState(const EmptyExtendedState& other)
    {
    }
    EmptyExtendedState(EmptyExtendedState&& other) noexcept
    {
    }
    EmptyExtendedState& operator=(const EmptyExtendedState& other)
    {
        return *this;
    }
    EmptyExtendedState& operator=(EmptyExtendedState&& other) noexcept
    {
        return *this;
    }
    void console() const
    {
	std::cout << "empty extended state" << std::endl;
    }
    template <class Archive>
    void serialize(Archive& archive)
    {
        archive(foo);
    }
    char foo;
};

/**
 * @brief Encapsulate the current robot state, the desired robot state
 * and an optional extended state. It also encapsulate
 * the current iteration, frequency and time stamp.
 * An instance of Observation is immutable.
 * Observations are created and written in the share memory
 * by instances of BackEnd at each iteration,
 * and read from the shared memory by instances of FrontEnd.
 * @tparam NB_ACTUATORS the number of actuators of the robot
 * @tparam ROBOT_STATE the class used to encapsulate robot states
 * @tparam EXTENDED_STATE arbitrary class used to encapsulate arbitrary
 * information
 */
template <int NB_ACTUATORS, class ROBOT_STATE, class EXTENDED_STATE>
class Observation
{
public:
    Observation();

    void copy(
        const Observation<NB_ACTUATORS, ROBOT_STATE, EXTENDED_STATE>& from,
        bool full);
    Observation(
        const Observation<NB_ACTUATORS, ROBOT_STATE, EXTENDED_STATE>& other);
    Observation(Observation<NB_ACTUATORS, ROBOT_STATE, EXTENDED_STATE>&&
                    other) noexcept;

    Observation<NB_ACTUATORS, ROBOT_STATE, EXTENDED_STATE>& operator=(
        const Observation<NB_ACTUATORS, ROBOT_STATE, EXTENDED_STATE>& other);

    Observation<NB_ACTUATORS, ROBOT_STATE, EXTENDED_STATE>& operator=(
        Observation<NB_ACTUATORS, ROBOT_STATE, EXTENDED_STATE>&&
            other) noexcept;

    Observation(States<NB_ACTUATORS, ROBOT_STATE> observed_state,
                States<NB_ACTUATORS, ROBOT_STATE> desired_state,
                long int stamp,
                long int iteration,
                double frequency);

    Observation(States<NB_ACTUATORS, ROBOT_STATE> observed_state,
                States<NB_ACTUATORS, ROBOT_STATE> desired_state,
                long int stamp,
                long int control_iteration,
                long int sensor_iteration,
                double frequency);

    Observation(States<NB_ACTUATORS, ROBOT_STATE> observed_state,
                States<NB_ACTUATORS, ROBOT_STATE> desired_state,
                EXTENDED_STATE extended_state,
                long int stamp,
                long int iteration,
                double frequency);

    Observation(States<NB_ACTUATORS, ROBOT_STATE> observed_state,
                States<NB_ACTUATORS, ROBOT_STATE> desired_state,
                EXTENDED_STATE extended_state,
                long int stamp,
                long int control_iteration,
                long int sensor_iteration,
                double frequency);

    /**
     * @brief returns the actual state of each actuator
     */
    const States<NB_ACTUATORS, ROBOT_STATE>& get_observed_states() const;

    /**
     * @brief returns the desired state of each actuator,
     * as computed by the backend based on the command queue, and
     * set to the robot
     */
    const States<NB_ACTUATORS, ROBOT_STATE>& get_desired_states() const;

    /**
     * @brief returns an instance of extended state, which is an
     * instance of an arbitrary class encaspulating arbitrary information
     */
    const EXTENDED_STATE& get_extended_state() const;

    /**
     * @brief robot control iteration corresponding to this
     * observation
     */
    long int get_control_iteration() const;

    /**
     * @brief robot sensor iteration corresponding to this
     * observation
     */
    long int get_sensor_iteration() const;

    /**
     * @brief backend iteration corresponding to this
     * observation
     */
    long int get_iteration() const;

    /**
     * @brief frequency of backend at the iteration
     * this observation was created
     */
    double get_frequency() const;

    /**
     * @brief returns a string description of the observation
     */
    std::string to_string() const;

    /**
     * @brief used internally to serialize the instances of
     * observation when written in the shared memory
     */
    template <class Archive>
    void serialize(Archive& archive)
    {
        archive(observed_states_,
                desired_states_,
                extended_state_,
                control_iteration_,
                sensor_iteration_,
                stamp_,
                iteration_,
                observed_frequency_);
    }

protected:
    friend shared_memory::private_serialization;

    States<NB_ACTUATORS, ROBOT_STATE> observed_states_;
    States<NB_ACTUATORS, ROBOT_STATE> desired_states_;
    EXTENDED_STATE extended_state_;
    long int control_iteration_;
    long int sensor_iteration_;
    long int stamp_;
    long int iteration_;
    double observed_frequency_;
};

#include "observation.hxx"
}
