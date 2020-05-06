// Copyright (c) 2019 Max Planck Gesellschaft
// Author : Vincent Berenz

#pragma once

#include <memory>
#include <vector>
#include "internal/commands_setter.hpp"
#include "internal/observation_exchange.hpp"
#include "synchronizer/leader.hpp"


namespace o80

{
// used internally by the front-end and the back-end to set
// the number of iterations the back-end should burst through
namespace internal
{
void set_bursting(const std::string &segment_id, int nb_iterations);
}

    typedef std::shared_ptr<synchronizer::Leader> LeaderPtr;

    
/**
 * @brief FrontEnd is the user interface communicating
 * with a BackEnd. It uses an interprocess shared memory
 * under the hood. "communicating" refers to writing to
 * a queue of command, and/or reading latest observations
 * written by the backend.
 * The front-end encapsulates a command queue, and add_command
 * functions add commands to this queue. Commands are transfered
 * from this local queue to the shared memory queue only when
 * specific function, such as "pulse" and "pulse_and_wait"
 * are called.
 * @tparam QUEUE_SIZE number of commands that can be hosted
 * in the command queue at any point of time. Exceptions will be
 * thrown if more commands are queued.
 * @tparam NB_ACTUATORS number of actuators of the robot
 * @tparam ROBOT_STATE class encapsulating the state of an
 * actuator of the robot
 * @tparam EXTENDED_STATE (optional) class encapsulating
 * supplementary arbitrary information
 */
template <int QUEUE_SIZE,
          int NB_ACTUATORS,
          class ROBOT_STATE,
          class EXTENDED_STATE>
class FrontEnd
{

public:
    
    typedef time_series::MultiprocessTimeSeries<Observation<NB_ACTUATORS,
							    ROBOT_STATE,
							    EXTENDED_STATE>> History;

    typedef std::vector<Observation<NB_ACTUATORS,
				    ROBOT_STATE,
				    EXTENDED_STATE>> HistoryChunk;

    
public:
    /**
     * @param segment_id should be the same for the
     * backend and the frontend
     */
    FrontEnd(std::string segment_id);

    int get_nb_actuators() const;
    
    time_series::Index get_current_iteration();
    bool update_history_since(time_series::Index iteration,
			   HistoryChunk& push_back_to);
    bool update_latest(size_t nb_items,
		    HistoryChunk& push_back_to);

    HistoryChunk get_history_since(time_series::Index iteration);
    HistoryChunk get_latest(size_t nb_items);
    Observation<NB_ACTUATORS, ROBOT_STATE, EXTENDED_STATE> wait_for_next();

    
    /**
     * @brief Add an iteration command to the local command queue.
     * "iteration" means this command aims at the robot to reach
     * at the specified target state when performing the target
     * iteration, performing interpolation from the state the actuator
     * is in when the command starts execution.
     * @param nb_actuators : the actuator targeted by this command
     * @param target_state : the target state corresponding to this command
     * @param iteration : the controller of the back-end will interpolate
     * between
     * its current starting state to the target state between its current
     * starting
     * iteration and this target iteration.
     * @param Mode : if OVERWRITE, wipe the content of the local and shared
     * memory
     * queue before adding this command to the queue.
     */
    void add_command(int nb_actuators,
                     ROBOT_STATE target_state,
                     Iteration target_iteration,
                     Mode mode);

    /**
     * @brief Add a direct command to the local command queue. "Direct"
     * means that this command will be executed during
     * a single iteration, and target state will become the desired state
     * of the specified actuator upon execution
     * of the command.
     * @param nb_actuators : the actuator targeted by this command
     * @param target_state : the target state corresponding to this command
     * @param Mode : if OVERWRITE, wipe the content of the local and shared
     * memory
     * queue before adding this command to the queue.
     */
    void add_command(int nb_actuators, ROBOT_STATE target_state, Mode mode);

    /**
     * @brief Add a speed command to the local command queue.
     * "speed" means this command aims at the actuator to reach
     * at the specified target state at the specified speed, performing
     * interpolation
     * from the state the actuator
     * is in when the command starts execution. The units of "speed" depends
     * on the interpolation function implemented by ROBOT_STATE.
     * @param nb_actuators : the actuator targeted by this command
     * @param target_state : the target state corresponding to this command
     * @param iteration : the controller of the back-end will interpolate
     * between
     * its current starting state to the target state between its current
     * starting
     * iteration and this target iteration.
     * @param Mode : if OVERWRITE, wipe the content of the local and shared
     * memory
     * queue before adding this command to the queue.
     */
    void add_command(int nb_actuator,
                     ROBOT_STATE target_state,
                     Speed speed,
                     Mode mode);

    /**
     * @brief Assumes the Backend is encapsulated in a
     * Standalone that has been started in burst mode, i.e.
     * it does iterate only when receiveing a signal from the front end.
     * This method is for sending such signal, which will trigger the backend
     * to run for nb_iterations iteration as fast as it can.
     * The frontend write the local queue of commands to the shared memory
     * queue of commands before sending the signal.
     * @param nb_iterations : the number of iteration the backend should perform
     */
    Observation<NB_ACTUATORS, ROBOT_STATE, EXTENDED_STATE> burst(
        int nb_iterations);

    void final_burst();

    /**
     * @brief write the local queue of commands to the shared memory queue, then
     * wait until the backend reaches the specified iteration. The read the
     * corresponding
     * observation from the shared memory and returns it.
     */
    Observation<NB_ACTUATORS, ROBOT_STATE, EXTENDED_STATE> pulse(
        Iteration iteration);

    /**
     * @brief write the local queue of commands to the shared memory queue, then
     * reads the latest
     * observation from the shared memory and returns it.
     */
    Observation<NB_ACTUATORS, ROBOT_STATE, EXTENDED_STATE> pulse();

    /**
     * @brief write the local queue of commands to the shared memory queue, then
     * waits for all these commands to be terminated. Then read the lastest
     * observation
     * from the shared memory and returns it.
     */
    Observation<NB_ACTUATORS, ROBOT_STATE, EXTENDED_STATE> pulse_and_wait();

    /**
     * @brief read the latest observation from the shared memory and returns
     * it.
     */
    Observation<NB_ACTUATORS, ROBOT_STATE, EXTENDED_STATE> read();

private:
    std::string segment_id_;

    time_series::Index history_index_;
    
    // in charge of reading observation from the shared memory
    ObservationExchange<NB_ACTUATORS, ROBOT_STATE, EXTENDED_STATE>
        observation_exchange_;

    // in charge of writting commmands to the shared memory
    CommandsSetter<QUEUE_SIZE, ROBOT_STATE> commands_setter_;

    // keeps in memory the ids of the latest commands written
    // to the shared memory. Used internally by "pulse_and_wait"
    // to detect when all commands have been achieved.
    // (commands_setter_ above reads from the shared memory
    // the ids of commands which have been completed)
    std::set<int> command_ids_;

    // in burst mode: used to the send the activating signal to the backend.
    LeaderPtr leader_;
};

#include "front_end.hxx"
}
