// Copyright (c) 2019 Max Planck Gesellschaft
// Author : Vincent Berenz

namespace o80
{
template <int NB_ACTUATORS, int QUEUE_SIZE, class STATE>
ControllersManager<NB_ACTUATORS, QUEUE_SIZE, STATE>::ControllersManager(
    std::string segment_id)
    : commands_{CommandsTimeSeries::create_leader(segment_id + "_commands",
                                                  QUEUE_SIZE)},
      commands_index_(-1),
      pulse_id_(0),
      completed_commands_{CompletedCommandsTimeSeries::create_leader(
          segment_id + "_completed", QUEUE_SIZE)},
      segment_id_(segment_id),
      relative_iteration_(-1),
      received_commands_{CompletedCommandsTimeSeries::create_leader(
          segment_id + "_received", QUEUE_SIZE)},
      starting_commands_{CompletedCommandsTimeSeries::create_leader(
          segment_id + "_starting", QUEUE_SIZE)}
{
    for (int i = 0; i < NB_ACTUATORS; i++)
    {
        initialized_[i] = false;
        controllers_[i].set_completed_commands(completed_commands_);
        controllers_[i].set_starting_commands(starting_commands_);
    }
    shared_memory::set<long int>(segment_id_, "pulse_id", pulse_id_);
    shared_memory::set<time_series::Index>(
        segment_id_, "command_read", commands_index_);
}

template <int NB_ACTUATORS, int QUEUE_SIZE, class STATE>
bool ControllersManager<NB_ACTUATORS, QUEUE_SIZE, STATE>::
    reapplied_desired_states() const
{
    for (int dof = 0; dof < NB_ACTUATORS; dof++)
    {
        if (!controllers_[dof].reapplied_desired_state())
        {
            return false;
        }
    }
    return true;
}

template <int NB_ACTUATORS, int QUEUE_SIZE, class STATE>
void ControllersManager<NB_ACTUATORS, QUEUE_SIZE, STATE>::purge()
{
    for (Controller<STATE>& controller : controllers_)
    {
        controller.purge();
    }
}

template <int NB_ACTUATORS, int QUEUE_SIZE, class STATE>
void ControllersManager<NB_ACTUATORS, QUEUE_SIZE, STATE>::process_commands(
    long int current_iteration)
{
    if (commands_.is_empty())
    {
        return;
    }

    // checking if the frontend is done with its current command batch
    long int current_pulse_id;
    shared_memory::get<long int>(segment_id_, "pulse_id", current_pulse_id);
    if (current_pulse_id == pulse_id_)
    {
        return;
    }

    time_series::Index newest_index = commands_.newest_timeindex(false);
    if (newest_index < commands_index_)
    {
        return;
    }

    if (commands_index_ == -1)
    {
        commands_index_ = commands_.oldest_timeindex(false);
    }

    for (time_series::Index index = commands_index_; index <= newest_index;
         index++)
    {
        Command<STATE> command = commands_[index];
        if (command.get_pulse_id() > current_pulse_id)
        {
            newest_index = index - 1;
            break;
        }
        CommandType& command_type = command.get_command_type();
        if (command_type.type == Type::ITERATION)
        {
            if (command_type.iteration.do_reset)
            {
                relative_iteration_ = current_iteration;
            }
            if (command_type.iteration.relative)
            {
                command_type.iteration.value += relative_iteration_;
            }
        }
        int dof = command.get_dof();
        if (dof < 0 || dof >= controllers_.size())
        {
            throw std::runtime_error("command with incorrect dof index");
        }
        received_commands_.append(command.get_id());
        controllers_[dof].set_command(command);
    }
    pulse_id_ = current_pulse_id;
    commands_index_ = newest_index + 1;
    shared_memory::set<time_series::Index>(
        segment_id_, "command_read", commands_index_);
}

template <int NB_ACTUATORS, int QUEUE_SIZE, class STATE>
STATE ControllersManager<NB_ACTUATORS, QUEUE_SIZE, STATE>::get_desired_state(
    int dof,
    long int current_iteration,
    const TimePoint& time_now,
    const STATE& current_state)
{
    if (dof < 0 || dof >= controllers_.size())
    {
        throw std::runtime_error("command with incorrect dof index");
    }

    if (!initialized_[dof])
    {
        previous_desired_states_.values[dof] = current_state;
        initialized_[dof] = true;
    }

    const STATE& desired = controllers_[dof].get_desired_state(
        current_iteration,
        current_state,
        previous_desired_states_.values[dof],
        time_now);

    previous_desired_states_.values[dof] = desired;
    return desired;
}

template <int NB_ACTUATORS, int QUEUE_SIZE, class STATE>
int ControllersManager<NB_ACTUATORS, QUEUE_SIZE, STATE>::get_current_command_id(
    int dof) const
{
    if (dof < 0 || dof >= controllers_.size())
    {
        throw std::runtime_error("command with incorrect dof index");
    }
    return controllers_[dof].get_current_command_id();
}

template <int NB_ACTUATORS, int QUEUE_SIZE, class STATE>
void ControllersManager<NB_ACTUATORS, QUEUE_SIZE, STATE>::
    get_newly_executed_commands(std::queue<int>& get)
{
    for (Controller<STATE>& controller : controllers_)
    {
        controller.get_newly_executed_commands(get);
    }
}

template <int NB_ACTUATORS, int QUEUE_SIZE, class STATE>
time_series::MultiprocessTimeSeries<Command<STATE>>&
ControllersManager<NB_ACTUATORS, QUEUE_SIZE, STATE>::get_commands_time_series()
{
    return commands_;
}

template <int NB_ACTUATORS, int QUEUE_SIZE, class STATE>
time_series::MultiprocessTimeSeries<int>&
ControllersManager<NB_ACTUATORS, QUEUE_SIZE, STATE>::
    get_completed_commands_time_series()
{
    return completed_commands_;
}
}  // namespace o80
