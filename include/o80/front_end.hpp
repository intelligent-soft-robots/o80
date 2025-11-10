// Copyright (c) 2019 Max Planck Gesellschaft
// Author : Vincent Berenz

#pragma once

#include <memory>
#include <set>
#include <vector>
#include "burster.hpp"
#include "o80_internal/command.hpp"
#include "observation.hpp"
#include "shared_memory/shared_memory.hpp"
#include "synchronizer/leader.hpp"
#include "time_series/multiprocess_time_series.hpp"
#include "time_series/time_series.hpp"

namespace o80

{
/*! A frontend sends commands to a related backend and
 *  read observations writen by this same backend.
 * @tparam QUEUE_SIZE size of the commands and observations
           time series
 * @tparam NB_ACTUATORS number of actuators of the robot
 * @tparam ROBOT_STATE class encapsulating the state of an
 *                     actuator of the robot
 * @tparam EXTENDED_STATE class encapsulating
 *                        supplementary arbitrary information
 */
template <int QUEUE_SIZE,
          int NB_ACTUATORS,
          class ROBOT_STATE,
          class EXTENDED_STATE>
class FrontEnd
{
public:
    /*! multiprocess time series hosting commands shared with the backend*/
    typedef time_series::MultiprocessTimeSeries<Command<ROBOT_STATE>>
        CommandsTimeSeries;
    /*! time series buffering commands before their transfer
        to the commands time series*/
    typedef time_series::TimeSeries<Command<ROBOT_STATE>>
        BufferCommandsTimeSeries;
    /*! multiprocess times series hosting the commands id that have been
        processed by the backend*/
    typedef time_series::MultiprocessTimeSeries<int>
        CompletedCommandsTimeSeries;
    /*! multprocess time series hosting the observations writen by the backend*/
    typedef time_series::MultiprocessTimeSeries<
        Observation<NB_ACTUATORS, ROBOT_STATE, EXTENDED_STATE>>
        ObservationsTimeSeries;
    /*! vector of observations*/
    typedef std::vector<Observation<NB_ACTUATORS, ROBOT_STATE, EXTENDED_STATE>>
        Observations;
    /*! for bursting mode support */
    typedef std::shared_ptr<BursterClient> BursterClientPtr;

public:
    /**
     * @param segment_id should be the same for the
     *        backend and the frontend
     */
    FrontEnd(std::string segment_id);

    ~FrontEnd();

    // TODO: to revive
    // void start_logging(std::string logger_segment_id);

    /*! Returns the frequency at which the backend is set to run.
      This will return a value only if the backend has been instantiated
      by a standalone. Otherwise, throws a runtime_error
    */
    float get_frequency() const;

    /*!returns the number of actuators*/
    int get_nb_actuators() const;

    /*! Read from the shared memory all the observations
        starting from the specified iteration until the newest
        iteration and update the observations vector with them.
     *  @param[in] iteration: iteration number of the backend
     *  @param[out] push_back_to: vector of observations to be updated.
     */
    bool observations_since(time_series::Index iteration,
                            Observations& push_back_to);

    /*! Read from the shared memory
        the latest nb_items observations
        update the observations vector with them.
     *  @param[in] iteration: iteration number of the backend
     *  @param[out] push_back_to: vector of observations to be updated.
     */
    bool update_latest_observations(size_t nb_items,
                                    Observations& push_back_to);

    /*! Returns a vector of observations containing all observations
     *  starting from the specified iteration until the latest iteration
     *  @param iteration: iteration number
     */
    Observations get_observations_since(time_series::Index iteration);

    /*! Returns a vector of observations containing the nb_items
     *  latest  observations.
     *  @param iteration: number of observations to read
     */
    Observations get_latest_observations(size_t nb_items);

    /*! waiting for the next observation to be writen by the backend, then
     *  returning it. During the first call to this method, the current
     *  iteration is initialized as reference iteration, then the reference
     *  iteration will be increase by one at each call*/
    Observation<NB_ACTUATORS, ROBOT_STATE, EXTENDED_STATE> wait_for_next();

    /*! if returns true: during its latest iteration, the backend did not
     * reapply the previous desired states (i.e. at least one command was
     * active), if false, the backend reapplied the previous desired state (no
     * active command)
     */
    bool backend_is_active();

    /*! reset the reference iteration used by the "wait_for_next" method
     *  to the current iteration number*/
    void reset_next_index();

    /*! add a command to the buffer commands time series.*/
    void add_command(int nb_actuator,
                     ROBOT_STATE target_state,
                     Iteration target_iteration,
                     Mode mode);

    /*! add a command to the buffer commands time series.*/
    void add_command(int nb_actuator,
                     ROBOT_STATE target_state,
                     Duration_us duration,
                     Mode mode);

    /*! add a command to the buffer commands time series.*/
    void add_command(int nb_actuators, ROBOT_STATE target_state, Mode mode);

    /*! add a command to the buffer commands time series.*/
    void add_command(int nb_actuator,
                     ROBOT_STATE target_state,
                     Speed speed,
                     Mode mode);

    /*! add to each actuator an overwriting command with
     *  the initial state (as returned by the initial_states method)
     *  as target state  */
    void add_reinit_command();

    /*! requests the backend to purge its command queues (including current
      commands) during next iteration*/
    void purge() const;

    /*! request the related backend or standalone to perform nb_iterations
        in a row, as fast as possible. Assumes the related backend or standalone
        is running in bursting mode*/
    Observation<NB_ACTUATORS, ROBOT_STATE, EXTENDED_STATE> burst(
        int nb_iterations);

    /*! Will trigger the related standalone to run one more iteration then
        exit. Assumes the standalone runs in bursting mode.
     */
    void final_burst();

    /*! write all buffered commands to the multiprocess time series commands
     *  (i.e. the related backend will read and execute them), then wait until
     *  the backend reaches the specified iteration, before returning the
     *  observation related to this iteration.
     */
    Observation<NB_ACTUATORS, ROBOT_STATE, EXTENDED_STATE> pulse(
        Iteration iteration);

    /*! write all buffered commands to the multiprocess time series commands
     *  (i.e. the related backend will read and execute them), then return
     * the latest observation/
     */
    Observation<NB_ACTUATORS, ROBOT_STATE, EXTENDED_STATE> pulse();

    /*! write all buffered commands to the multiprocess time series commands
     *  (i.e. the related backend will read and execute them), then return
     * the latest observation once all commands have been completed
     */
    Observation<NB_ACTUATORS, ROBOT_STATE, EXTENDED_STATE> pulse_and_wait();

    /*! write all buffered commands to the multiprocess time series commands
     *  (i.e. the related backend will read and execute them), then return
     * the latest observation. Optionaly, this call may be followed by a
     * call to wait, which will be blocking until completion of all commands
     */
    Observation<NB_ACTUATORS, ROBOT_STATE, EXTENDED_STATE> pulse_prepare_wait();

    /*! If the previous call to this instance of frontend was a call to
     * "pulse_prepare_wait", a call to wait will be blocking until
     * all commands added to the multiprocess time series commands have
     * been executed. If following call to another of the "pulse" method,
     * thows a runtime_error.
     */
    Observation<NB_ACTUATORS, ROBOT_STATE, EXTENDED_STATE> wait();

    /*! write all buffered commands to the multiprocess time series commands
     *  (i.e. the related backend will read and execute them), then wait for the
     *  backend to finish executation of all commands, then return the latest
     *  observation.
     */
    Observation<NB_ACTUATORS, ROBOT_STATE, EXTENDED_STATE> read(
        long int iteration = -1);

    /*!
     * returns the first states ever observed by the backend
     */
    States<NB_ACTUATORS, ROBOT_STATE> initial_states() const;

public:
    /*! returns the time series of commands shared between the
     *   frontend and the backend*/
    static auto get_introspection_commands(std::string segment_id);

    /*! returns the time series of (completed) command ids
     *  shared between the frontend and the backend*/
    static auto get_introspection_completed_commands(std::string segment_id);

    /*! returns the time series of command ids the frontend
     *  waits completion of */
    static auto get_introspection_waiting_for_completion(
        std::string segment_id);

    /*! returns the time series of command ids the frontend
     *  processed reports of completion */
    static auto get_introspection_completion_reported(std::string segment_id);

private:
    void size_check();
    time_series::Index last_index_read_by_backend();
    void share_commands(std::set<int>& command_ids, bool store);
    void wait_for_completion(std::set<int>& command_ids,
                             time_series::Index completed_index);

private:
    // to delete !
    void _print(CommandsTimeSeries* time_series);

    std::string segment_id_;

    time_series::Index history_index_;

    // used to write commands to the shared memory
    CommandsTimeSeries commands_;
    // tracking ids of commands shared by this frontend.
    // used by the "pulse_and_wait" method.
    std::set<int> sent_command_ids_;

    // used to sync frontend and backend
    // (making sure all command "pulsed" at the same time
    // are read by the backend at the same iteration)
    long int pulse_id_;

    // used to buffer commands before writing them
    // to the shared memory
    BufferCommandsTimeSeries buffer_commands_;
    time_series::Index buffer_index_;

    // backend will write observation into it
    ObservationsTimeSeries observations_;
    time_series::Index observations_index_;

    // backend will write into it completed commands
    // used by the method "pulse_and_wait" (i.e. waiting
    // for shared commands to be completed)
    CompletedCommandsTimeSeries completed_commands_;

    // everytime the frontend will wait for the completion of a
    // command (pulse_and_wait method), it will write the corresponding id in
    // this time series. For debug and introspection.
    CompletedCommandsTimeSeries waiting_for_completion_;

    // everytime the frontend will process the information that a
    // command has been completed by the backend, its id will be
    // written in this time series. For debug and introspection.
    CompletedCommandsTimeSeries completion_reported_;

    // for the use of prepare_wait
    int completed_index_;
    bool wait_prepared_;

    // for bursting mode support
    BursterClientPtr burster_client_;
};

#include "front_end.hxx"
}  // namespace o80
