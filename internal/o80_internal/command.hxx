// Copyright (C) 2019 Max Planck Gesellschaft
// Author : Vincent Berenz

namespace o80
{
template <class STATE>
int Command<STATE>::id;

template <class STATE>
int Command<STATE>::get_next_id()
{
    std::lock_guard<std::mutex> guard(Command<STATE> mutex);
    Command<STATE>::id++;
    return Command<STATE>::id;
}

template <class STATE>
Command<STATE>::Command()
    : pulse_id_(-1),
      target_state_(),
      id_(-1),
      dof_(-1),
      mode_(Mode::OVERWRITE),
      command_status_(),
      command_type_()
{
}

template <class STATE>
void Command<STATE>::copy(const Command<STATE>& from, bool full)
{
    if (full)
    {
        pulse_id_ = from.pulse_id_;
        target_state_ = from.target_state_;
        command_status_ = from.command_status_;
    }
    id_ = from.id_;
    mode_ = from.mode_;
    dof_ = from.dof_;
    command_type_ = from.command_type_;
}

template <class STATE>
Command<STATE>::Command(const Command<STATE>& other)
    : command_type_(other.command_type_)
{
    copy(other, true);
}

template <class STATE>
Command<STATE>::Command(Command<STATE>&& other) noexcept
    : command_type_(std::move(other.command_type_)),
      target_state_(std::move(other.target_state_))
{
    copy(other, false);
}

template <class STATE>
Command<STATE>& Command<STATE>::operator=(const Command<STATE>& other)
{
    copy(other, true);
    return *this;
}

template <class STATE>
Command<STATE>& Command<STATE>::operator=(Command<STATE>&& other) noexcept
{
    copy(other, false);
    target_state_ = std::move(other.target_state_);
    command_status_ = std::move(other.command_status_);
    return *this;
}

template <class STATE>
Command<STATE>::Command(long int pulse_id,
                        STATE target_state,
                        Duration_us duration,
                        int dof,
                        Mode mode)

    : pulse_id_(pulse_id),
      target_state_(target_state),
      dof_(dof),
      mode_(mode),
      command_status_(),
      command_type_(duration)
{
    id_ = Command<STATE>::get_next_id();
}

template <class STATE>
Command<STATE>::Command(
    long int pulse_id, STATE target_state, Speed speed, int dof, Mode mode)

    : pulse_id_(pulse_id),
      target_state_(target_state),
      dof_(dof),
      mode_(mode),
      command_status_(),
      command_type_(speed)
{
    id_ = Command<STATE>::get_next_id();
}

template <class STATE>
Command<STATE>::Command(long int pulse_id,
                        STATE target_state,
                        Iteration iteration,
                        int dof,
                        Mode mode)
    : pulse_id_(pulse_id),
      target_state_(target_state),
      dof_(dof),
      mode_(mode),
      command_status_(),
      command_type_(iteration)
{
    id_ = Command<STATE>::get_next_id();
}

template <class STATE>
Command<STATE>::Command(long int pulse_id,
                        STATE target_state,
                        int dof,
                        Mode mode)
    : pulse_id_(pulse_id),
      target_state_(target_state),
      dof_(dof),
      mode_(mode),
      command_status_(),
      command_type_()
{
    id_ = Command<STATE>::get_next_id();
}

template <class STATE>
void Command<STATE>::print() const
{
    std::cout << "command " << id_;
    std::cout << " | dof: " << dof_;
    if (mode_ == Mode::OVERWRITE)
    {
        std::cout << " | mode: overwrite";
    }
    else
    {
        std::cout << " | mode: queue";
    }
    if (command_type_.type == Type::DIRECT)
    {
        std::cout << " | type: direct";
    }
    else if (command_type_.type == Type::DURATION)
    {
        std::cout << " | type: duration";
        std::cout << " | duration (microseconds): "
                  << command_type_.duration.value;
    }
    else if (command_type_.type == Type::ITERATION)
    {
        std::cout << " | type: iteration";
        std::cout << " | iteration: " << command_type_.iteration.value;
    }
    else
    {
        std::cout << " | type: speed";
        std::cout << " | speed: " << command_type_.speed.value;
    }
    std::cout << "\n";
}

template <class STATE>
int Command<STATE>::get_id() const
{
    return id_;
}

template <class STATE>
bool Command<STATE>::operator<(const Command& other) const
{
    return id_ < other.get_id();
}

template <class STATE>
bool Command<STATE>::operator>(const Command& other) const
{
    return id_ > other.get_id();
}

template <class STATE>
const STATE& Command<STATE>::get_target_state() const
{
    return target_state_;
}

template <class STATE>
int Command<STATE>::get_dof() const
{
    return dof_;
}

template <class STATE>
Mode Command<STATE>::get_mode() const
{
    return mode_;
}

template <class STATE>
const CommandStatus<STATE>& Command<STATE>::get_command_status() const
{
    return command_status_;
}

template <class STATE>
CommandStatus<STATE>& Command<STATE>::get_mutable_command_status()
{
    return command_status_;
}

template <class STATE>
CommandType& Command<STATE>::get_command_type()
{
    return command_type_;
}

template <class STATE>
long int Command<STATE>::get_pulse_id() const
{
    return pulse_id_;
}

template <class STATE>
std::mutex Command<STATE>::mutex;
}
