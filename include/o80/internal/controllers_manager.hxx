// Copyright (c) 2019 Max Planck Gesellschaft
// Author : Vincent Berenz

namespace o80
{
template <int NB_ACTUATORS, int QUEUE_SIZE, class STATE>
ControllersManager<NB_ACTUATORS, QUEUE_SIZE, STATE>::ControllersManager(std::string segment_id)
    :  commands_(segment_id+"_commands",QUEUE_SIZE,true),
       commands_index_(-1),
       completed_commands_(segment_id+"_completed",QUEUE_SIZE,true),
       segment_id_(segment_id)
{
    for (int i = 0; i < NB_ACTUATORS; i++)
    {
        initialized_[i] = false;
	controllers_[i].set_completed_commands(completed_commands_);
    }
    shared_memory::set<time_series::Index>(segment_id_,
					   "command_read",
					   commands_index_);
}

template <int NB_ACTUATORS, int QUEUE_SIZE, class STATE>
bool ControllersManager<NB_ACTUATORS, QUEUE_SIZE, STATE>::reapplied_desired_states() const
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
void ControllersManager<NB_ACTUATORS, QUEUE_SIZE, STATE>::process_commands()
{
  
    time_series::Index newest_index = commands_.newest_timeindex(false);
    if(newest_index<=commands_index_)
	{
	    return;
	}
    if(commands_index_==-1)
      {
	commands_index_ = commands_.oldest_timeindex(false);
      }
    for(time_series::Index index=commands_index_;
	index<=newest_index;index++)
	{
	    Command<STATE> command = commands_[index];
	    int dof = command.get_dof();
	    if (dof < 0 || dof >= controllers_.size())
		{
		    throw std::runtime_error("command with incorrect dof index");
		}
	    controllers_[dof].set_command(command);
	}
    commands_index_=newest_index;
    shared_memory::set<time_series::Index>(segment_id_,
					   "command_read",
					   commands_index_);
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
void ControllersManager<NB_ACTUATORS, QUEUE_SIZE, STATE>::get_newly_executed_commands(
    std::queue<int>& get)
{
    for (Controller<STATE>& controller : controllers_)
    {
        controller.get_newly_executed_commands(get);
    }
}


template <int NB_ACTUATORS, int QUEUE_SIZE, class STATE>
time_series::MultiprocessTimeSeries<Command<STATE>>&
ControllersManager<NB_ACTUATORS, QUEUE_SIZE, STATE>::get_commands_time_series(){
  return commands_;
}

template <int NB_ACTUATORS, int QUEUE_SIZE, class STATE>
time_series::MultiprocessTimeSeries<int>&
ControllersManager<NB_ACTUATORS, QUEUE_SIZE, STATE>::get_completed_commands_time_series(){
  return completed_commands_;
}

  
}

