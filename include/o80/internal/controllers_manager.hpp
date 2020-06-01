// Copyright (C) 2019 Max Planck Gesellschaft
// Author : Vincent Berenz

#pragma once

#include <memory>
#include "time_series/multiprocess_time_series.hpp"
#include "command.hpp"
#include "controller.hpp"
#include "o80/states.hpp"

namespace o80
{
    template <int NB_ACTUATORS, int QUEUE_SIZE, class STATE>
class ControllersManager
{
public:
    typedef std::array<Controller<STATE>, NB_ACTUATORS> Controllers;
    typedef time_series::MultiprocessTimeSeries<Command<STATE>> CommandsTimeSeries;
    typedef time_series::MultiprocessTimeSeries<int> CompletedCommandsTimeSeries;
public:
    ControllersManager(std::string segment_id);

    void process_commands(long int current_iteration);
	
    STATE get_desired_state(int dof,
                            long int current_iteration,
                            const TimePoint &time_now,
                            const STATE &current_state);

    int get_current_command_id(int dof) const;

    void get_newly_executed_commands(std::queue<int> &get);

    bool reapplied_desired_states() const;

  CommandsTimeSeries& get_commands_time_series();
  CompletedCommandsTimeSeries& get_completed_commands_time_series();
  
private:
    std::string segment_id_;
    CommandsTimeSeries commands_;
    long int pulse_id_;
    time_series::Index commands_index_;
    CompletedCommandsTimeSeries completed_commands_;
    Controllers controllers_;
    States<NB_ACTUATORS, STATE> previous_desired_states_;
    std::array<bool, NB_ACTUATORS> initialized_;
  long int relative_iteration_;
};
}

#include "controllers_manager.hxx"
