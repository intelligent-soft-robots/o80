// Copyright (C) 2019 Max Planck Gesellschaft
// Author : Vincent Berenz

#pragma once

#include <deque>
#include <memory>
#include <set>
#include "command.hpp"
#include "command_id.hpp"
#include "o80/type.hpp"
#include "shared_memory/exchange_manager_consumer.hpp"
#include "shared_memory/exchange_manager_producer.hpp"

namespace o80
{
template <int QUEUE_SIZE, class STATE>
class CommandsSetter
{
public:
    CommandsSetter(std::string segment_id, std::string object_id);
    ~CommandsSetter();

public:
    /**
     * Request the client to write a direct command to the shared memory.
     * Note: commands are queueed in a buffer, and a given number of commands
     *       are written to the shared memory everytime the "communicate"
     *       function is called.
     * @param dof : degree of freedom this command refers to
     * @sign Sign::PLUS for agonist PAM, Sign::MINUS for antagonist PAM
     * @target_state
     * @mode Mode::QUEUE to add the command on top of the queue
     *       of commands for the specific PAM (dof and sign), i.e. executation
     *       once all previously requested commands have been terminated,
     *       or Mode::OVERWRITE to clear up the queue (including
     *       currently running command, which will be interrupted)
     *       and start executation of this command as soon as possible
     * @return the id of the command
     */
    int add_command(int dof, STATE target_state, Mode mode);

    int add_command(int dof,
                    STATE target_state,
                    Iteration iteration,
                    Mode mode);

    /**
     * Request the client to write a direct command to the shared memory.
     * The stamp parameter allows to delay execution in the future.
     * Note: commands are queueed in a buffer, and a given number of commands
     *       are written to the shared memory everytime the "communicate"
     *       function is called.
     * @param dof : degree of freedom this command refers to
     * @sign Sign::PLUS for agonist PAM, Sign::MINUS for antagonist PAM
     * @target_state
     * @param stamp. The relative time (in microsecond) in the future at which
     * the command
     *               at which the command should be executed. The reference time
     *               is set at "now" the first time this function is called.
     *               If the relative time is negative, or has passed, then the
     *               command is executed immediately.
     * @mode Mode::QUEUE to add the command on top of the queue
     *       of commands for the specific PAM (dof and sign), i.e. executation
     *       once all previously requested commands have been terminated,
     *       or Mode::OVERWRITE to clear up the queue (including
     *       currently running command, which will be interrupted)
     *       and start executation of this command as soon as possible
     * @return the id of the command
     */
    int add_command(int dof, STATE target_state, TimeStamp stamp, Mode mode);

    /**
     * Request the client to write a duration command to the shared memory.
     * Note: commands are queueed in a buffer, and a given number of commands
     *       are written to the shared memory everytime the "communicate"
     *       function is called.
     * @param dof : degree of freedom this command refers to
     * @sign Sign::PLUS for agonist PAM, Sign::MINUS for antagonist PAM
     * @target_state
     * @param duration. Warning: if the mode used is QUEUE (see below),
              then it may be unclear from which current state the
              command will be executed, and if the target state, and the
              current state turn out to be quite different, and the duration
              quite low, this will result in a violent motion.
              You may prefer to use a speed command.
     * @mode Mode::QUEUE to add the command on top of the queue
     *       of commands for the specific PAM (dof and sign), i.e. executation
     *       once all previously requested commands have been terminated,
     *       or Mode::OVERWRITE to clear up the queue (including
     *       currently running command, which will be interrupted)
     *       and start executation of this command as soon as possible
     * @return the id of the command
     */
    int add_command(int dof,
                    STATE target_state,
                    std::chrono::microseconds duration,
                    Mode mode);

    /**
     * Request the client to write a speed command to the shared memory.
     * Note: commands are queueed in a buffer, and a given number of commands
     *       are written to the shared memory everytime the "communicate"
     *       function is called.
     * @param dof : degree of freedom this command refers to
     * @sign Sign::PLUS for agonist PAM, Sign::MINUS for antagonist PAM
     * @target_state
     * @param : speed, in units of state per second
     * @mode Mode::QUEUE to add the command on top of the queue
     *       of commands for the specific PAM (dof and sign), i.e. executation
     *       once all previously requested commands have been terminated,
     *       or Mode::OVERWRITE to clear up the queue (including
     *       currently running command, which will be interrupted)
     *       and start executation of this command as soon as possible
     * @return the id of the command
     */
    int add_command(int dof, STATE target_state, Speed speed, Mode mode);

    void go_to_posture(const std::map<int, STATE> &dof_state,
                       int speed,
                       long int time_precision);

    /**
     * The "communicate" function will be called iteratively
     *  until command_id has been completed
     * @param: command_id, id of the command, as returned by
               "add_command"
     * @param: precision: period at which "communicate" will be called
     */
    void wait_for_completion(int command_id,
                             std::chrono::microseconds precision);

    /**
     *  The "communicate" function will be called iteratively
     *  until all command_ids have been completed
     * @param: command_ids: set of commands, as returned by
               "add_command"
     * @param: precision: period at which "communicate" will be called
     */
    bool wait_for_completion(std::set<int> &command_ids,
                             std::chrono::microseconds precision);

    /**
     * write commands to the shared_memory, and
     * read data from the shared memory.
     */
    bool communicate();

private:
    // shared memory segment and object ids
    std::string segment_id_;
    std::string object_id_;
    std::string completed_segment_id_;
    std::string completed_object_id_;
    std::string command_object_id_;

private:
    shared_memory::Exchange_manager_producer<Command<STATE>, QUEUE_SIZE>
        running_commands_exchange_;
    shared_memory::Exchange_manager_consumer<CommandId, QUEUE_SIZE>
        completed_commands_exchange_;
    std::set<int> non_completed_commands_;
    std::deque<Command<STATE>> commands_buffer_;
};

#include "commands_setter.hxx"
}
