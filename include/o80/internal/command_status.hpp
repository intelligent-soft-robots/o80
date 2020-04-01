// Copyright (C) 2019 Max Planck Gesellschaft
// Author : Vincent Berenz

#pragma once

#include "command_type.hpp"
#include "o80/mode.hpp"
#include "o80/type.hpp"

namespace o80
{
template <class STATE>
class CommandStatus

{
public:
    CommandStatus();
    CommandStatus(const CommandStatus<STATE>& other);
    CommandStatus(CommandStatus<STATE>&& other) noexcept;
    CommandStatus<STATE>& operator=(const CommandStatus<STATE>& other);
    CommandStatus<STATE>& operator=(CommandStatus<STATE>&& other) noexcept;

    bool set_initial_conditions(long int starting_iteration,
                                const STATE& starting_state,
                                const STATE& target_state,
                                const TimePoint& start_time,
                                const CommandType& command_type);

public:
    const TimePoint& get_start_time() const;
    long int get_start_iteration() const;
    const STATE& get_starting_state() const;
    bool is_active() const;
    void set_active();
    void set_inactive();
    void set_direct_done();
    bool finished(long int current_iteration,
                  const TimePoint& now,
                  const STATE& starting,
                  const STATE& current,
                  const STATE& previous_desired,
                  const STATE& target) const;
    const Type& get_type() const;
    const CommandType& get_command_type() const;

private:
    // for shared_memory::serializer to be able to serialize
    // a command into a string (used internally by exchange_manager)
    friend shared_memory::private_serialization;

private:
    void copy(const CommandStatus<STATE>& from, bool full);

private:
    STATE starting_state_;
    TimePoint starting_time_;
    long int starting_iteration_;
    bool initialized_;
    bool active_;
    bool direct_done_;
    CommandType command_type_;
};
}

#include "command_status.hxx"
