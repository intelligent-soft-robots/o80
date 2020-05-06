// Copyright (C) 2019 Max Planck Gesellschaft
// Author : Vincent Berenz

#pragma once

#include <chrono>
#include <mutex>
#include <queue>
#include "command.hpp"
#include "command_status.hpp"
#include "command_type.hpp"
#include "o80/typedefs.hpp"

namespace o80
{
/* Base class for controllers. This API should not be used directly,
   but rather through an instance of client.
   To create your own controller, inheritate this class and
   implement the get_desired_pressure_ function. See
   Linear_controller as example.
   @see Linear_controller
   @see Client
 */

template <class STATE>
class Controller
{
public:
    Controller();

    void set_command(const Command<STATE>& command);

    bool stop_current(const STATE& current_state,
                      Microseconds control_iteration);

    void stop_all(const STATE& current_state, Microseconds control_iteration);

    int running(const STATE& current_state, Microseconds control_iteration);

    int size() const;

    const STATE& get_desired_state(long int current_iteration,
                                   const STATE& current_state,
                                   const STATE& previous_desired_state,
                                   const TimePoint& time_now);

    int get_current_command_id() const;
    void get_newly_executed_commands(std::queue<int>& q);

    bool reapplied_desired_state() const;
    
private:
    // control iteration is used to remove invalid commands, i.e. commands
    // that would require to change pressure in a duration smaller than one
    // control iteration period
    Command<STATE>* get_current_command(long int current_iteration,
                                        const STATE& current_state,
                                        const STATE& previously_desired_state,
                                        const TimePoint& time_now);

    void reset();

private:
    static std::mutex mutex_;

    std::queue<Command<STATE>> queue_;
    Command<STATE> current_command_;
    std::queue<int> executed_commands_;
    STATE desired_state_;
    const STATE* current_state_;
    // memory weather or not the latest called to
    // get_desired_state was based on command execution,
    // or returning the same desired state (i.e. command
    // queue was empty)
    bool reapplied_desired_state_;
};
}

#include "controller.hxx"
