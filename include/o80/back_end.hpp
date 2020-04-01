// Copyright (c) 2019 Max Planck Gesellschaft
// Author : Vincent Berenz

#pragma once

#include "o80/internal/commands_getter.hpp"
#include "o80/internal/controllers_manager.hpp"
#include "o80/internal/observation_exchange.hpp"
#include "o80/states.hpp"
#include "real_time_tools/realtime_check.hpp"

namespace o80

{
/**
 * @brief delete the related shared memory segment.
 * Creation of new instance of BackEnd pointing
 * to a non cleaned up shared memory segments may hang.
 * Shared memory segments are cleanup upon destruction of
 * BackEnd instances, this function is to be used as backup
 * if a program hosting a BackEnd crashed, and the
 * related destructor failed to be called.
 */
void clear_shared_memory(std::string segment_id);

/**
 * BackEnd is the entity managing the commands sent
 * by a FrontEnd. BackEnd manages a queue of commands
 * (one queue per actuator)BackEnd reads from the shared memory
 * new commands, and at each call to "iterate" uses this queue
 * to compute for each actuator the next desired state to set to the robot.
 * At each iteration, the backend communicates with the shared memory:
 * 1) reads new commands (written by the front end), 2) read current state
 * from the robot and write it; and 3) write the id of completed commands.
 * @tparam QUEUE_SIZE number of commands that can be hosted
 * in the command queue at any point of time. Exceptions will be
 * thrown if more commands are queued.
 * @tparam NB_ACTUATORS number of actuators of the robot
 * @tparam STATE class encapsulating the state of an
 * actuator of the robot
 * @tparam EXTENDED_STATE (optional) class encapsulating
 * supplementary arbitrary information
 */
template <int QUEUE_SIZE, int NB_ACTUATORS, class STATE, class EXTENDED_STATE>
class BackEnd
{
public:
    /**
     * @param segment_id should be the same for the
     * backend and the frontend
     */
    BackEnd(std::string segment_id);

    /**
     * @brief delete the shared memory segment
     * Note that running front end based on the same
     * segment_id will not be able to connect to
     * new instances of BackEnd reusing this segment id.
     * Once a Backend has been destroyed, related FrontEnd
     * should also be terminated.
     */
    ~BackEnd();

    /**
     * The backend iterates once.
     * @param time_now : current time stamp in microseconds
     * @param current_states : current state for each actuator
     * @param extended_state : arbitrary information to be added to the
     * Observation
     * that will be written in the shared memory during the iteration
     * @return the desired states for each actuator, based on the current queue
     * of commands
     */
    const States<NB_ACTUATORS, STATE>& pulse(
        const TimePoint& time_now,
        const States<NB_ACTUATORS, STATE>& current_states,
        const EXTENDED_STATE& extended_state,
        bool iteration_update = true,
        long int current_iteration = -1,
        bool print_observation = true);

    /**
     * The backend iterates once.
     * @param time_now : current time stamp in microseconds
     * @param current_states : current state for each actuator
     * @return the desired states for each actuator, based on the current queue
     * of commands
     */
    const States<NB_ACTUATORS, STATE>& pulse(
        const TimePoint& time_now,
        const States<NB_ACTUATORS, STATE>& current_states,
        bool iteration_update = true,
        long int current_iteration = -1,
        bool print_observation = true);

private:
    // performing on iteration. Called internally by "pulse"
    void iterate(const TimePoint& time_now,
                 const States<NB_ACTUATORS, STATE>& current_states,
                 bool iteration_update = true,
                 long int current_iteration = -1);

private:
    // id of shared memory segment
    std::string segment_id_;

    // used to read the commands from the shared memory
    CommandsGetter<QUEUE_SIZE, STATE> commands_getter_;

    // host controllers (one per actuator), each controller compute
    // the current desired state based on its current command
    ControllersManager<NB_ACTUATORS, STATE> controllers_manager_;

    // used to write Observation in the shared memory
    ObservationExchange<NB_ACTUATORS, STATE, EXTENDED_STATE>
        observation_exchange_;

    // Desired states as computed by the controllers. Reference to this instance
    // is returned by "pulse" functions
    States<NB_ACTUATORS, STATE> desired_states_;

    // queue of commands. New commands are added by commands_getter,
    // and commands are poped by controllers_manager
    std::queue<Command<STATE>> commands_;

    // queue of completed command ids, written in the shared memory
    // commands ids are added by controllers_manager
    std::queue<int> completed_commands_;

    // incremented at each call of iterate
    long int iteration_;

    // compute the current frequency (frequency of call to iterate),
    // related frequency is written in the Observation
    real_time_tools::RealTimeCheck frequency_measure_;
    double observed_frequency_;
};

#include "back_end.hxx"
}
