// Copyright (C) 2019 Max Planck Gesellschaft
// Author : Vincent Berenz

namespace o80
{
template <class STATE>
Controller<STATE>::Controller()
    : current_state_(nullptr), reapplied_desired_state_(true)
{
}

template <class STATE>
void Controller<STATE>::share_completed_command(const Command<STATE>& command)
{
    completed_commands_->append(command.get_id());
}

template <class STATE>
void Controller<STATE>::reset()
{
    CommandStatus<STATE>& command_status =
        current_command_.get_mutable_command_status();

    if (command_status.is_active())
    {
        share_completed_command(current_command_);
        command_status.set_inactive();
    }

    while (!queue_.empty())
    {
        share_completed_command(queue_.front());
        queue_.pop();
    }
}

template <class STATE>
void Controller<STATE>::set_completed_commands(
    CompletedCommandsTimeSeries& completed_commands)
{
    completed_commands_ = &completed_commands;
}

template <class STATE>
void Controller<STATE>::set_command(const Command<STATE>& command)
{
    std::lock_guard<std::mutex> guard(mutex_);

    Mode mode = command.get_mode();

    if (mode == Mode::OVERWRITE)
    {
        reset();
    }
    queue_.push(command);
}

template <class STATE>
Command<STATE>* Controller<STATE>::get_current_command(
    long int current_iteration,
    const STATE& current_state,
    const STATE& previously_desired_state,
    const TimePoint& time_now)
{
    std::lock_guard<std::mutex> guard(mutex_);

    {
        // if there is a current command, check
        // if finished. if not, returning it
        CommandStatus<STATE>& command_status =
            current_command_.get_mutable_command_status();

        if (command_status.is_active())
        {
            return &(current_command_);
        }
    }

    // nothing going on
    if (queue_.empty())
    {
        return NULL;
    }

    // getting top (lower command_id) command
    current_command_ = queue_.front();

    queue_.pop();

    {
        CommandStatus<STATE>& command_status =
            current_command_.get_mutable_command_status();
        const CommandType& command_type = current_command_.get_command_type();

        // initializing it, if requested
        bool valid_command = command_status.set_initial_conditions(
            current_iteration,
            previously_desired_state,
            current_command_.get_target_state(),
            time_now,
            command_type);

        if (!valid_command)
        {
            // this may happen for speed command requiring to go to a state too
            // close to
            // current state, resulting in a duration that is below control
            // iteration, therefore
            // not applicable. Just removing the command from the queue.
            // (which should be fine from the user perspective, as target state
            // is almost
            // current state)
            share_completed_command(current_command_);
            command_status.set_inactive();
            return NULL;
        }

        command_status.set_active();
    }

    return &(current_command_);
}

template <class STATE>
int Controller<STATE>::get_current_command_id() const
{
    std::lock_guard<std::mutex> guard(mutex_);

    const CommandStatus<STATE>& status = current_command_.get_command_status();

    if (!status.is_active())
    {
        return -1;
    }

    return current_command_.get_id();
}

template <class STATE>
bool Controller<STATE>::stop_current(
    const STATE& current_state, std::chrono::microseconds control_iteration)
{
    bool initialized;

    {
        std::lock_guard<std::mutex> guard(mutex_);

        const Command<STATE>* command =
            get_current_command(current_state, control_iteration);

        if (command == NULL)
        {
            return false;
        }

        current_command_.set_inactive();
        return true;
    }
}

template <class STATE>
void Controller<STATE>::stop_all(const STATE& current_state,
                                 std::chrono::microseconds control_iteration)
{
    bool stopped = true;
    while (stopped)
    {
        stopped = stop_current(current_state, control_iteration);
    }
}

template <class STATE>
int Controller<STATE>::running(const STATE& current_state,
                               std::chrono::microseconds control_iteration)
{
    const Command<STATE>* command =
        get_current_command(current_state, control_iteration);

    if (command == NULL)
    {
        return -1;
    }

    return command->get_id();
}

template <class STATE>
int Controller<STATE>::size() const
{
    return queue_.size();
}

template <class STATE>
bool Controller<STATE>::reapplied_desired_state() const
{
    return reapplied_desired_state_;
}

template <class STATE>
const STATE& Controller<STATE>::get_desired_state(
    long int current_iteration,
    const STATE& current_state,
    const STATE& previously_desired_state,
    const TimePoint& time_now)
{
    // getting current commmand
    Command<STATE>* command = get_current_command(
        current_iteration, current_state, previously_desired_state, time_now);

    reapplied_desired_state_ = false;
    if (command == NULL)
    {
        reapplied_desired_state_ = true;
        return previously_desired_state;
    }

    CommandStatus<STATE>& command_status =
        command->get_mutable_command_status();
    const Type& type = command_status.get_type();
    const CommandType& command_type = command_status.get_command_type();

    // if direct command, we do not need a controller,
    // we just need to apply the target state
    if (type == Type::DIRECT)
    {
        const STATE& state = command->get_target_state();
        command_status.set_direct_done();
        share_completed_command(*command);
        command_status.set_inactive();
        return state;
    }

    // if STATE is a sublcass of SensorState, then it is not expected
    // to get interpolation and finished method (i.e only direct commands
    // are supported)
    if constexpr (!std::is_base_of<SensorState, STATE>::value)
    {
        const STATE& starting_state = command_status.get_starting_state();
        const STATE& target_state = command->get_target_state();
        const TimePoint& start_time = command_status.get_start_time();

        if (type == Type::SPEED)
        {
            desired_state_ =
                target_state.intermediate_state(start_time,
                                                time_now,
                                                starting_state,
                                                current_state,
                                                previously_desired_state,
                                                target_state,
                                                command_type.speed);
        }

        if (type == Type::DURATION)
        {
            desired_state_ =
                target_state.intermediate_state(start_time,
                                                time_now,
                                                starting_state,
                                                current_state,
                                                previously_desired_state,
                                                target_state,
                                                command_type.duration);
        }

        if (type == Type::ITERATION)
        {
            long int start_iteration = command_status.get_start_iteration();
            desired_state_ =
                target_state.intermediate_state(start_iteration,
                                                current_iteration,
                                                starting_state,
                                                current_state,
                                                previously_desired_state,
                                                target_state,
                                                command_type.iteration);
        }

        // checking if current command finished, and updating
        // its status accordingly. If command finished,
        // poping the next one
        if (command_status.finished(current_iteration,
                                    time_now,
                                    command_status.get_starting_state(),
                                    current_state,
                                    previously_desired_state,
                                    current_command_.get_target_state()))
        {
            share_completed_command(current_command_);
            command_status.set_inactive();
            get_current_command(current_iteration + 1,
                                current_state,
                                previously_desired_state,
                                time_now);
        }

        return desired_state_;
    }
}

template <class STATE>
std::mutex Controller<STATE>::mutex_;
}  // namespace o80
