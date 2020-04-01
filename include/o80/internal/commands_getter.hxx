// Copyright (c) 2019 Max Planck Gesellschaft
// Author : Vincent Berenz

#define TEMPLATE_CG template <int QUEUE_SIZE, class STATE>
#define COMMANDSGETTER CommandsGetter<QUEUE_SIZE, STATE>

TEMPLATE_CG
COMMANDSGETTER::CommandsGetter(std::string segment_id, std::string object_id)
    : segment_id_(segment_id),
      object_id_(object_id),
      completed_segment_id_("completed_" + segment_id),
      completed_object_id_("completed_" + object_id),
      command_object_id_("command_id"),
      commands_exchange_(segment_id, object_id, false, false),
      completed_commands_exchange_(
          completed_segment_id_, completed_object_id_, false, false)
{
    // this will be used to track which is the latest command id received.
    // so new command setter may increment command ids from this number
    shared_memory::set<int>(segment_id, command_object_id_, 0);
}

TEMPLATE_CG
void COMMANDSGETTER::read_commands_from_memory(std::queue<Command<STATE>> &get)
{
    if (commands_exchange_.ready_to_consume())
    {
        commands_exchange_.lock();

        while (true)
        {
            Command<STATE> command;
            bool consuming = commands_exchange_.consume(command);
            if (consuming)
            {
                if (command.get_id() >= 0)
                {
                    shared_memory::set<int>(
                        segment_id_, command_object_id_, command.get_id());
                    get.push(command);
                }
            }
            else
            {
                break;
            }
        }

        commands_exchange_.unlock();
    }
}

TEMPLATE_CG
void COMMANDSGETTER::write_completed_commands_to_memory(
    std::queue<int> &completed_commands)
{
    if (completed_commands_exchange_.ready_to_produce())
    {
        completed_commands_exchange_.lock();
        while (!completed_commands.empty())
        {
            try
            {
                completed_commands_exchange_.set(completed_commands.front());
                completed_commands.pop();
            }
            catch (shared_memory::Memory_overflow_exception)
            {
                break;
            }
        }
        completed_commands_exchange_.unlock();
    }
}
