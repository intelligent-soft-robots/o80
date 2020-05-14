// Copyright (c) 2019 Max Planck Gesellschaft
// Author : Vincent Berenz

#define TEMPLATE_BACKEND        \
    template <int QUEUE_SIZE,   \
              int NB_ACTUATORS, \
              class STATE,      \
              class EXTENDED_STATE>

#define BACKEND BackEnd<QUEUE_SIZE, NB_ACTUATORS, STATE, EXTENDED_STATE>

void clear_shared_memory(std::string segment_id)
{
    shared_memory::clear_shared_memory(segment_id);
    shared_memory::clear_shared_memory(std::string("completed_") + segment_id);
    shared_memory::clear_shared_memory(segment_id +
                                       std::string("_synchronizer"));
    shared_memory::clear_shared_memory(segment_id +
                                       std::string("_synchronizer_follower"));
    shared_memory::clear_shared_memory(segment_id +
                                       std::string("_synchronizer_leader"));
    // mutex cleaned on destruction
    shared_memory::Mutex m1(segment_id + std::string("_locker"), true);
    shared_memory::Mutex m2(
        std::string("completed_") + segment_id + std::string("_locker"), true);
}

TEMPLATE_BACKEND
BACKEND::BackEnd(std::string segment_id, bool new_commands_observations)
    : segment_id_(segment_id),
      commands_getter_(segment_id, std::string("commands")),
      controllers_manager_(),
      observation_exchange_(segment_id, std::string("observations"),
			    QUEUE_SIZE,true),
      desired_states_(),
      iteration_(0),
      observed_frequency_(-1),
      new_commands_observations_(new_commands_observations),
      logger_(nullptr)
{
    frequency_measure_.tick();
}

TEMPLATE_BACKEND
BACKEND::~BackEnd()
{
    clear_shared_memory(segment_id_);
    if (logger_!=nullptr)
      {
	delete logger_;
      }
}

TEMPLATE_BACKEND
void
BACKEND::start_logging(std::string logger_segment_id)
{
  logger_ = new Logger(QUEUE_SIZE,logger_segment_id,false);
}


TEMPLATE_BACKEND
bool BACKEND::iterate(const TimePoint& time_now,
                      const States<NB_ACTUATORS, STATE>& current_states,
                      bool iteration_update,
                      long int current_iteration)
{
    if (iteration_update)
    {
        iteration_++;
    }
    else
    {
        iteration_ = current_iteration;
    }

    commands_getter_.read_commands_from_memory(commands_);

    if(logger_!=nullptr)
      {
	logger_->log(segment_id_,LogAction::BACKEND_READ);
      }
    
    // dispatching commands to controllers
    while (!commands_.empty())
    {
        controllers_manager_.add_command(commands_.front());
        commands_.pop();
    }

    // reading desired state based on controllers output
    for (int controller_nb = 0; controller_nb < desired_states_.values.size();
         controller_nb++)
    {
        desired_states_.values[controller_nb] =
            controllers_manager_.get_desired_state(
                controller_nb,
                iteration_,
                time_now,
                current_states.values[controller_nb]);
    }

    // writte completed commands to memory
    controllers_manager_.get_newly_executed_commands(completed_commands_);
    commands_getter_.write_completed_commands_to_memory(completed_commands_);

    observed_frequency_ = frequency_measure_.tick();

    return controllers_manager_.reapplied_desired_states();
}

TEMPLATE_BACKEND
const States<NB_ACTUATORS, STATE>& BACKEND::pulse(
    const TimePoint& time_now,
    const States<NB_ACTUATORS, STATE>& current_states,
    const EXTENDED_STATE& extended_state,
    bool iteration_update,
    long int current_iteration)
{
    bool reapplied_desired_states = iterate(time_now, current_states,
				       iteration_update, current_iteration);

    bool print_obs = true;
    if (new_commands_observations_)
	{
	    if(reapplied_desired_states)
		{
		    print_obs=false;
		}
	}

    // writting current states to shared memory
    if (print_obs)
    {
        Observation<NB_ACTUATORS, STATE, EXTENDED_STATE> observation(
            current_states,
            desired_states_,
            extended_state,
            time_now.count(),
            iteration_,
            observed_frequency_);
        observation_exchange_.write(observation);
	if(logger_!=nullptr)
	  {
	    if (reapplied_desired_states)
	      {
		logger_->log(segment_id_,LogAction::BACKEND_WRITE_REAPPLY);
	      }
	    else
	      {
		logger_->log(segment_id_,LogAction::BACKEND_WRITE_NEW);
	      }
	  }
    }

    return desired_states_;
}

