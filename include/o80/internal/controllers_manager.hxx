// Copyright (c) 2019 Max Planck Gesellschaft
// Author : Vincent Berenz

namespace o80
{
template <int NB_ACTUATORS, class STATE>
ControllersManager<NB_ACTUATORS, STATE>::ControllersManager()
{
    for (int i = 0; i < NB_ACTUATORS; i++)
    {
        initialized_[i] = false;
    }
}

template <int NB_ACTUATORS, class STATE>
void ControllersManager<NB_ACTUATORS, STATE>::add_command(
    const Command<STATE>& command)
{
    int dof = command.get_dof();
    if (dof < 0 || dof >= controllers_.size())
    {
        throw std::runtime_error("command with incorrect dof index");
    }
    controllers_[dof].set_command(command);
}

template <int NB_ACTUATORS, class STATE>
STATE ControllersManager<NB_ACTUATORS, STATE>::get_desired_state(
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

template <int NB_ACTUATORS, class STATE>
int ControllersManager<NB_ACTUATORS, STATE>::get_current_command_id(
    int dof) const
{
    if (dof < 0 || dof >= controllers_.size())
    {
        throw std::runtime_error("command with incorrect dof index");
    }
    return controllers_[dof].get_current_command_id();
}

template <int NB_ACTUATORS, class STATE>
void ControllersManager<NB_ACTUATORS, STATE>::get_newly_executed_commands(
    std::queue<int>& get)
{
    for (Controller<STATE>& controller : controllers_)
    {
        controller.get_newly_executed_commands(get);
    }
}
};
