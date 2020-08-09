// Copyright (C) 2019 Max Planck Gesellschaft
// Author : Vincent Berenz

#pragma once

#include <chrono>
#include <iostream>
#include <mutex>
#include <vector>

#include "command_status.hpp"
#include "command_type.hpp"
#include "o80/command_types.hpp"
#include "o80/mode.hpp"
#include "shared_memory/serializer.hpp"
#include "shared_memory/shared_memory.hpp"
#include "time_stamp.hpp"

namespace o80
{
template <class STATE>
class Command
{
public:
    Command();

    Command(const Command<STATE>& other);
    Command(Command<STATE>&& other) noexcept;
    Command<STATE>& operator=(const Command<STATE>& other);
    Command<STATE>& operator=(Command<STATE>&& other) noexcept;

    // speed command, i.e. reaching the desired state
    // at given "unit of state" per second
    Command(
        long int pulse_id, STATE target_state, Speed speed, int dof, Mode mode);

    // duration command, i.e. reaching desired state at time now+duration
    Command(long int pulse_id,
            STATE target_state,
            Duration_us duration_us,
            int dof,
            Mode mode);

    // Iteration command, i.e. reaching desired state at
    // specified o80 backend iteration
    Command(long int pulse_id,
            STATE target_state,
            Iteration iteration,
            int dof,
            Mode mode);

    // Direct command, i.e. desired state becomes
    // target state immediately
    Command(long int pulse_id, STATE target_state, int dof, Mode mode);

    int get_id() const;
    const STATE& get_target_state() const;
    int get_dof() const;
    Mode get_mode() const;
    CommandType& get_command_type();
    long int get_pulse_id() const;
    void print() const;

private:
    void copy(const Command<STATE>& from, bool full);

public:
    // used for ensuring commands are executed in order
    // of id, i.e. in order of creation
    bool operator<(const Command& other) const;
    bool operator>(const Command& other) const;

public:
    // for exchange_manager to be able to serialize a
    // command
    template <class Archive>
    void serialize(Archive& archive)
    {
        // note: the command_status_, which is used for
        // monitoring of the execution of the command by a controller,
        // is excluded
        archive(pulse_id_, target_state_, id_, mode_, dof_, command_type_);
    }

private:
    // for shared_memory::serializer to be able to serialize
    // a command into a string (used internally by exchange_manager)
    friend shared_memory::private_serialization;

    long int pulse_id_;
    STATE target_state_;
    int id_;
    Mode mode_;
    int dof_;
    CommandType command_type_;

public:
    // Each command will have a unique id.
    // This id is a simple int, and a new command's id
    // is 'previously instantiated command's id+1'
    // But the very first command id used by the frontend when
    // creating its first command is not 0 or 1, rather
    // a value read from the shared memory (via this function).
    // This shared memory id is maintained by the backend, and
    // allows new frontend to start creating command id
    // at the command id of the previous frontend
    static void init_id(std::string segment_id, std::string object_id);

private:
    // used by the command constructors to attribute
    // the correct id to new commands
    static int get_next_id();
    static std::mutex mutex;
    static int id;

    CommandStatus<STATE> command_status_;

public:
    const CommandStatus<STATE>& get_command_status() const;
    CommandStatus<STATE>& get_mutable_command_status();
};
}  // namespace o80

#include "command.hxx"
