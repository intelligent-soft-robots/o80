// Copyright (c) 2019 Max Planck Gesellschaft
// Author : Vincent Berenz

#pragma once

#include <type_traits>
#include "o80/frequency_measure.hpp"
#include "o80/logger.hpp"
#include "o80/memory_clearing.hpp"
#include "o80/observation.hpp"
#include "o80/sensor_state.hpp"
#include "o80/states.hpp"
#include "o80_internal/controllers_manager.hpp"
#include "time_series/multiprocess_time_series.hpp"

namespace o80

{
/*!  The backend computes the desired state for each
 *   actuator, based on the time series of commands filled up
 *   by a frontend. The backend writes information to the time series
 *   of observations.
 *   @tparam QUEUE_SIZE number of commands that can be hosted
 *           in the command queue at any point of time. Exceptions will be
 *           thrown if more commands are queued.
 *   @tparam NB_ACTUATORS number of actuators of the robot
 *   @tparam STATE class encapsulating the state of an
 *           actuator of the robot
 *   @tparam EXTENDED_STATE class encapsulating
 *           supplementary arbitrary information */
template <int QUEUE_SIZE, int NB_ACTUATORS, class STATE, class EXTENDED_STATE>
class BackEnd
{
public:
    /*! Multiprocess time series hosting observations*/
    typedef time_series::MultiprocessTimeSeries<
        Observation<NB_ACTUATORS, STATE, EXTENDED_STATE>>
        ObservationsTimeSeries;

public:
    /**
     * @param segment_id should be the same for the
     *        backend and the frontend
     * @param new_commands_observations (default false).
     *        If true, information will be writen in the observation
     *        only when the desired state of any actuator changed
     *        (when false: an observation is writen for each iteration)
     */
    BackEnd(std::string segment_id, bool new_commands_observations = false);

    /**
     * @brief delete the shared memory segments
     */
    ~BackEnd();

    /**
     * The backend iterates once.
     * @param time_now : current time stamp in nanoseconds
     * @param current_states : current state for each actuator
     * @param extended_state : arbitrary information to be added to the
     *                         Observation that will be writen to the
     *                         observations time series.
     * @return : the desired states for each actuator, based on the current
     * queue
     *           of commands
     */
    const States<NB_ACTUATORS, STATE>& pulse(
        const TimePoint& time_now,
        const States<NB_ACTUATORS, STATE>& current_states,
        const EXTENDED_STATE& extended_state,
        bool iteration_update = true,
        long int current_iteration = -1);

    // TODO: revive
    // void start_logging(std::string logger_segment_id);

private:
    // performing on iteration. Called internally by "pulse"
    bool iterate(const TimePoint& time_now,
                 const States<NB_ACTUATORS, STATE>& current_states,
                 bool iteration_update = true,
                 long int current_iteration = -1);

private:
    // id of shared memory segments
    std::string segment_id_;

    // multiprocess time series, the backend write in it,
    // the frontends read from it
    ObservationsTimeSeries observations_;

    // host controllers (one per actuator), each controller compute
    // the current desired state based commands read from a commands
    // multiprocess time series.
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

    // TODO: revive
    // Logger* logger_;
};

#include "back_end.hxx"
}  // namespace o80
