// Copyright (c) 2019 Max Planck Gesellschaft
// Author : Vincent Berenz

#pragma once

#include <queue>

#include "command.hpp"
#include "shared_memory/exchange_manager_consumer.hpp"
#include "shared_memory/exchange_manager_producer.hpp"

namespace o80
{
template <int QUEUE_SIZE, class STATE>
class CommandsGetter
{
    typedef shared_memory::Exchange_manager_consumer<Command<STATE>, QUEUE_SIZE>
        Commands_exchange;
    typedef shared_memory::Exchange_manager_producer<int, QUEUE_SIZE>
        Completed_commands_exchange;

public:
    CommandsGetter(std::string segment_id, std::string object_id);

    void read_commands_from_memory(std::queue<Command<STATE>> &get);
    void write_completed_commands_to_memory(
        std::queue<int> &completed_commands);

private:
    std::string segment_id_;
    std::string object_id_;
    std::string completed_segment_id_;
    std::string completed_object_id_;
    std::string command_object_id_;

protected:
    Commands_exchange commands_exchange_;
    Completed_commands_exchange completed_commands_exchange_;
    std::queue<int> completed_commands_;
};

#include "commands_getter.hxx"
}
