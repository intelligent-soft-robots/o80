// Copyright (C) 2019 Max Planck Gesellschaft
// Author : Vincent Berenz

template <int QUEUE_SIZE, class STATE>
CommandsSetter<QUEUE_SIZE, STATE>::CommandsSetter(std::string segment_id,
                                                  std::string object_id)

    : segment_id_(segment_id),
      object_id_(object_id),
      completed_segment_id_("completed_" + segment_id),
      completed_object_id_("completed_" + object_id),
      command_object_id_("command_id"),
      running_commands_exchange_(segment_id_, object_id_, false, false),
      completed_commands_exchange_(
          completed_segment_id_, completed_object_id_, false, false)
{
    // reading from the backend from which command id
    // we should start
    Command<STATE>::init_id(segment_id_, "command_id");
}

template <int QUEUE_SIZE, class STATE>
CommandsSetter<QUEUE_SIZE, STATE>::~CommandsSetter()
{
}

template <int QUEUE_SIZE, class STATE>
int CommandsSetter<QUEUE_SIZE, STATE>::add_command(int dof,
                                                   STATE target_pressure,
                                                   Mode mode)
{
    Command<STATE> command(target_pressure, dof, mode);
    commands_buffer_.push_back(command);
    int id = command.get_id();
    non_completed_commands_.insert(id);
    return id;
}

template <int QUEUE_SIZE, class STATE>
int CommandsSetter<QUEUE_SIZE, STATE>::add_command(int dof,
                                                   STATE target_pressure,
                                                   Iteration iteration,
                                                   Mode mode)
{
    Command<STATE> command(target_pressure, iteration, dof, mode);
    commands_buffer_.push_back(command);
    int id = command.get_id();
    non_completed_commands_.insert(id);
    return id;
}

template <int QUEUE_SIZE, class STATE>
int CommandsSetter<QUEUE_SIZE, STATE>::add_command(int dof,
                                                   STATE target_pressure,
                                                   TimeStamp stamp,
                                                   Mode mode)
{
    Command<STATE> command(target_pressure, stamp, dof, mode);
    commands_buffer_.push_back(command);
    int id = command.get_id();
    non_completed_commands_.insert(id);
    return id;
}

template <int QUEUE_SIZE, class STATE>
int CommandsSetter<QUEUE_SIZE, STATE>::add_command(
    int dof,
    STATE target_pressure,
    std::chrono::microseconds duration,
    Mode mode)
{
    Command<STATE> command(target_pressure, duration, dof, mode);
    commands_buffer_.push_back(command);
    int id = command.get_id();
    non_completed_commands_.insert(id);
    return id;
}

template <int QUEUE_SIZE, class STATE>
int CommandsSetter<QUEUE_SIZE, STATE>::add_command(int dof,
                                                   STATE target_pressure,
                                                   Speed speed,
                                                   Mode mode)
{
    Command<STATE> command(target_pressure, speed, dof, mode);
    commands_buffer_.push_back(command);
    int id = command.get_id();
    non_completed_commands_.insert(id);
    return id;
}

template <int QUEUE_SIZE, class STATE>
void CommandsSetter<QUEUE_SIZE, STATE>::go_to_posture(
    const std::map<int, STATE> &dof_states, int speed, long int time_precision)
{
    std::chrono::microseconds precision(time_precision);
    std::set<int> command_ids;

    for (const std::pair<int, STATE> &dof_state : dof_states)
    {
        int id = this->add_command(
            dof_state.first, dof_state.second, speed, Mode::OVERWRITE);
        command_ids.insert(id);
    }

    this->wait_for_completion(command_ids, precision);
}

template <int QUEUE_SIZE, class STATE>
void CommandsSetter<QUEUE_SIZE, STATE>::wait_for_completion(
    int command_id, std::chrono::microseconds precision)
{
    while (true)
    {
        usleep(precision.count());
        communicate();

        int count = this->non_completed_commands_.count(command_id);
        if (count == 0)
        {
            break;
        }
    }
}

template <int QUEUE_SIZE, class STATE>
bool CommandsSetter<QUEUE_SIZE, STATE>::wait_for_completion(
    std::set<int> &command_ids, std::chrono::microseconds precision)
{
    static std::deque<int> to_remove;
    to_remove.clear();

    bool everything_shared = true;

    while (true && everything_shared)
    {
        usleep(precision.count());
        everything_shared = communicate();

        for (int command_id : command_ids)
        {
            int count = this->non_completed_commands_.count(command_id);
            if (count == 0)
            {
                to_remove.push_back(command_id);
            }
        }
        for (int command_id : to_remove)
        {
            command_ids.erase(command_id);
        }
        to_remove.clear();
        if (command_ids.size() == 0)
        {
            break;
        }
    }

    return everything_shared;
}

template <int QUEUE_SIZE, class STATE>
bool CommandsSetter<QUEUE_SIZE, STATE>::communicate()
{
    bool everything_shared = true;

    if (running_commands_exchange_.ready_to_produce())
    {
        running_commands_exchange_.lock();
        while (!commands_buffer_.empty())
        {
            try
            {
                everything_shared =
                    running_commands_exchange_.set(commands_buffer_.front());
                if (!everything_shared)
                {
                    break;
                }
                commands_buffer_.pop_front();
            }
            catch (const shared_memory::Memory_overflow_exception &e)
            {
                everything_shared = false;
                break;
            }
        }
        running_commands_exchange_.unlock();
    }

    if (completed_commands_exchange_.ready_to_consume())
    {
        completed_commands_exchange_.lock();
        while (true)
        {
            CommandId id;
            bool consuming = completed_commands_exchange_.consume(id);
            if (consuming)
            {
                non_completed_commands_.erase(id.value);
            }
            else
            {
                break;
            }
        }
        completed_commands_exchange_.unlock();
    }

    return everything_shared;
}
