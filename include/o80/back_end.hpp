// Copyright (c) 2019 Max Planck Gesellschaft
// Author : Vincent Berenz

#pragma once

#include "o80/frequency_measure.hpp"
#include "o80/internal/controllers_manager.hpp"
#include "o80/logger.hpp"
#include "o80/memory_clearing.hpp"
#include "o80/observation.hpp"
#include "o80/states.hpp"
#include "time_series/multiprocess_time_series.hpp"

namespace o80

{
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
    typedef time_series::MultiprocessTimeSeries<
        Observation<NB_ACTUATORS, STATE, EXTENDED_STATE>>
        ObservationsTimeSeries;

public:
    /**
     * @param segment_id should be the same for the
     * backend and the frontend
     */
    BackEnd(std::string segment_id, bool new_commands_observations = false);

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
        long int current_iteration = -1);

    void start_logging(std::string logger_segment_id);

private:
    // performing on iteration. Called internally by "pulse"
    bool iterate(const TimePoint& time_now,
                 const States<NB_ACTUATORS, STATE>& current_states,
                 bool iteration_update = true,
                 long int current_iteration = -1);

private:
    // id of shared memory segment
    std::string segment_id_;

    ObservationsTimeSeries observations_;

    // host controllers (one per actuator), each controller compute
    // the current desired state based on its current command
    ControllersManager<NB_ACTUATORS, QUEUE_SIZE, STATE> controllers_manager_;

    // Desired states as computed by the controllers. Reference to this instance
    // is returned by "pulse" functions
    States<NB_ACTUATORS, STATE> desired_states_;

    // incremented at each call of iterate
    long int iteration_;

    // compute the current frequency (frequency of call to iterate),
    // related frequency is written in the Observation
    FrequencyMeasure frequency_measure_;
    double observed_frequency_;

    // if true (default is false), only observation generated
    // a new commands execution times are shared
    // via the shared memory
    bool new_commands_observations_;

    Logger* logger_;
};

#include "back_end.hxx"
}
