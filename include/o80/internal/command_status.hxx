// Copyright (c) 2019 Max Planck Gesellschaft
// Author : Vincent Berenz

namespace o80
{
template <class STATE>
CommandStatus<STATE>::CommandStatus()
    : starting_state_(), active_(false), direct_done_(false)
{
}

template <class STATE>
void CommandStatus<STATE>::copy(const CommandStatus<STATE>& from, bool full)
{
    if (full)
    {
        starting_state_, from.starting_state_;
    }
    starting_time_ = from.starting_time_;
    initialized_ = from.initialized_;
    active_ = from.active_;
    direct_done_ = from.direct_done_;
}

template <class STATE>
CommandStatus<STATE>::CommandStatus(const CommandStatus<STATE>& other)
{
    copy(other, true);
}

template <class STATE>
CommandStatus<STATE>& CommandStatus<STATE>::operator=(
    const CommandStatus<STATE>& other)
{
    copy(other, true);
    return *this;
}

template <class STATE>
CommandStatus<STATE>::CommandStatus(CommandStatus<STATE>&& other) noexcept
    : CommandStatus()
{
    copy(other, false);
    starting_state_ = std::move(other.starting_state_);
}

template <class STATE>
CommandStatus<STATE>& CommandStatus<STATE>::operator=(
    CommandStatus<STATE>&& other) noexcept
{
    copy(other, false);
    starting_state_ = std::move(other.starting_state_);
    return *this;
}

template <class STATE>
const TimePoint& CommandStatus<STATE>::get_start_time() const
{
    return starting_time_;
}

template <class STATE>
long int CommandStatus<STATE>::get_start_iteration() const
{
    return starting_iteration_;
}

template <class STATE>
const STATE& CommandStatus<STATE>::get_starting_state() const
{
    return starting_state_;
}

// minimal duration_us is used because if starting pressure is too close to
// target pressure,
// (or equal to), then duration will be smaller than a control iteration, and
// not achievable. In this case, this method returns false, i.e. the command
// should
// be considered as invalid and simply removed from the queue
template <class STATE>
bool CommandStatus<STATE>::set_initial_conditions(
    long int starting_iteration,
    const STATE& starting_state,
    const STATE& target_state,
    const TimePoint& start_time,
    const CommandType& command_type)
{
    command_type_ = command_type;
    starting_iteration_ = starting_iteration;
    starting_time_ = start_time;
    starting_state_ = starting_state;
    initialized_ = true;

    if (command_type.type == Type::DIRECT)
    {
        return true;
    }

    else if (command_type.type == Type::SPEED)
    {
        if (command_type.speed.value == 0.0)
        {
            return false;
        }
    }

    else if (command_type.type == Type::ITERATION)
    {
        if (command_type.iteration.value <= starting_iteration)
        {
            return false;
        }
    }

    return true;
}

template <class STATE>
bool CommandStatus<STATE>::finished(long int current_iteration,
                                    const TimePoint& now,
                                    const STATE& starting,
                                    const STATE& current,
                                    const STATE& previous_desired,
                                    const STATE& target) const
{
    if (!initialized_)
    {
        throw std::runtime_error("Command_status has not been initialized");
    }

    if (command_type_.type == Type::ITERATION)
    {
        if (current_iteration > command_type_.iteration.value)
        {
            return true;
        }
        return false;
    }

    if (command_type_.type == Type::DIRECT)
    {
        if (direct_done_)
        {
            return true;
        }
        return false;
    }

    if (!initialized_)
    {
        return false;
    }

    if (command_type_.type == Type::SPEED)
    {
        return target.finished(get_start_time(),
                               now,
                               starting,
                               current,
                               previous_desired,
                               target,
                               command_type_.speed.value);
    }

    if (command_type_.type == Type::DURATION)
    {
        long int time_diff = o80::time_diff(get_start_time(), now);
        if (time_diff >= command_type_.duration.value)
        {
            return true;
        }
        return false;
    }

    return true;
}

template <class STATE>
bool CommandStatus<STATE>::is_active() const
{
    return active_;
}

template <class STATE>
void CommandStatus<STATE>::set_inactive()
{
    active_ = false;
}

template <class STATE>
void CommandStatus<STATE>::set_active()
{
    active_ = true;
}

template <class STATE>
void CommandStatus<STATE>::set_direct_done()
{
    direct_done_ = true;
}

template <class STATE>
const Type& CommandStatus<STATE>::get_type() const
{
    return command_type_.type;
}
template <class STATE>
const CommandType& CommandStatus<STATE>::get_command_type() const
{
    return command_type_;
}
}
