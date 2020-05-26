// Copyright (c) 2019 Max Planck Gesellschaft
// Author : Vincent Berenz

#pragma once

#include <memory>
#include <vector>
#include "internal/command.hpp"
#include "observation.hpp"
#include "synchronizer/leader.hpp"
#include "logger.hpp"
#include "time_series/multiprocess_time_series.hpp"
#include "time_series/time_series.hpp"

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
    
    typedef time_series::MultiprocessTimeSeries<Command<ROBOT_STATE>> CommandsTimeSeries;
    typedef time_series::TimeSeries<Command<ROBOT_STATE>> BufferCommandsTimeSeries;
    typedef time_series::MultiprocessTimeSeries<int> CompletedCommandsTimeSeries;
    typedef time_series::MultiprocessTimeSeries<Observation<NB_ACTUATORS,
							    ROBOT_STATE,
							    EXTENDED_STATE>> ObservationsTimeSeries;
    typedef std::vector<Observation<NB_ACTUATORS,
				    ROBOT_STATE,
				    EXTENDED_STATE>> Observations;

    
  public:
    /**
     * @param segment_id should be the same for the
     * backend and the frontend
     */
    FrontEnd(std::string segment_id);

    ~FrontEnd();
  
    void start_logging(std::string logger_segment_id);
  
    int get_nb_actuators() const;
    
    bool observations_since(time_series::Index iteration,
			      Observations& push_back_to);
    bool update_latest_observations(size_t nb_items,
		       Observations& push_back_to);
    Observations get_observations_since(time_series::Index iteration);
    Observations get_latest_observations(size_t nb_items);
    Observation<NB_ACTUATORS, ROBOT_STATE, EXTENDED_STATE> wait_for_next();
    void reset_next_index();
    
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

    void add_command(int nb_actuators,
                     ROBOT_STATE target_state,
                     Duration_us duration,
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

    void log(LogAction action);
    time_series::Index last_index_read_by_backend();
    void share_commands(std::set<int>& command_ids, bool store);
    void wait_for_completion(std::set<int>& command_ids);
    
  private:
    std::string segment_id_;

    time_series::Index history_index_;

    // used to write commands to the shared memory
    CommandsTimeSeries commands_;
    // tracking ids of commands shared by this frontend.
    // used by the "pulse_and_wait" method.
    std::set<int> sent_command_ids_;
    
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
    time_series::Index last_completed_command_index_;

    // in burst mode: used to the send the activating signal to the backend.
    LeaderPtr leader_;

    // used to log all event, e.g. commands written to shared memory, etc
    Logger* logger_;
  
  };

#include "front_end.hxx"
}
