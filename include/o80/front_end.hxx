

#define TEMPLATE_FRONTEND        \
    template <int QUEUE_SIZE,    \
              int NB_ACTUATORS,  \
              class ROBOT_STATE, \
              class EXTENDED_STATE>

#define FRONTEND FrontEnd<QUEUE_SIZE, NB_ACTUATORS, ROBOT_STATE, EXTENDED_STATE>

namespace internal
{
void set_bursting(const std::string &segment_id, int nb_iterations)
{
    shared_memory::set<int>(segment_id, "bursting", nb_iterations);
}
}

TEMPLATE_FRONTEND
FRONTEND::FrontEnd(std::string segment_id)
    : segment_id_(segment_id),
      commands_(segment_id+"_commands",QUEUE_SIZE,false),
      buffer_commands_(QUEUE_SIZE),
      buffer_index_(-1),
      observations_(segment_id+"_observations",QUEUE_SIZE,false),
      completed_commands_(segment_id+"_completed",QUEUE_SIZE,false),
      last_completed_command_index_(-1),
      leader_(nullptr),
      logger_(nullptr)
{
  observations_index_ = observations_.newest_timeindex(false);
  internal::set_bursting(segment_id, 1);
}

TEMPLATE_FRONTEND
FRONTEND::~FrontEnd()
{
  if(logger_!=nullptr)
    {
      delete logger_;
    }
}

TEMPLATE_FRONTEND
void
FRONTEND::log(LogAction action)
{
  if(logger_!=nullptr)
    {
      logger_->log(segment_id_,action);
    }
}

TEMPLATE_FRONTEND
void
FRONTEND::start_logging(std::string logger_segment_id)
{
  logger_ = new Logger(QUEUE_SIZE,logger_segment_id,false);
}


TEMPLATE_FRONTEND
int FRONTEND::get_nb_actuators() const
{
    return NB_ACTUATORS;
}

TEMPLATE_FRONTEND
Observation<NB_ACTUATORS, ROBOT_STATE, EXTENDED_STATE> FRONTEND::wait_for_next()
{
    observations_index_+=1;
    log(LogAction::FRONTEND_WAIT_START);
    observations_.wait_for_timeindex(observations_index_);
    Observation<NB_ACTUATORS,
		ROBOT_STATE,
		EXTENDED_STATE> obs =  observations_[observations_index_];
    log(LogAction::FRONTEND_WAIT_END);
    return obs;
}

TEMPLATE_FRONTEND
void FRONTEND::reset_next_index()
{
    observations_index_ = observations_.newest_timeindex(false)-1;
}

TEMPLATE_FRONTEND
bool FRONTEND::observations_since(time_series::Index time_index,
				 std::vector<Observation<NB_ACTUATORS,
				 ROBOT_STATE,
				 EXTENDED_STATE>>& v)
{
    time_series::Index oldest = observations_.oldest_timeindex();
    time_series::Index newest = observations_.newest_timeindex();
    if (time_index > newest || time_index<oldest)
	{
	    return false;
	}
    for(time_series::Index index=time_index; index<=newest; index++)
	{
	    v.push_back(observations_[index]);
	}
    return true;
}

TEMPLATE_FRONTEND
std::vector<Observation<NB_ACTUATORS,
			ROBOT_STATE,
			EXTENDED_STATE>>
FRONTEND::get_observations_since(time_series::Index time_index)
{
    std::vector<Observation<NB_ACTUATORS,
			    ROBOT_STATE,
			    EXTENDED_STATE>> v;
    observations_since(time_index,v);
    return v;
}

TEMPLATE_FRONTEND
bool FRONTEND::update_latest_observations(size_t nb_items,
			     std::vector<Observation<NB_ACTUATORS,
			     ROBOT_STATE,
			     EXTENDED_STATE>>& v)
{
    bool r=true;
    time_series::Index oldest = observations_.oldest_timeindex();
    time_series::Index newest = observations_.newest_timeindex();
    time_series::Index target = newest-nb_items+1;
    if (target<oldest)
	{
	    target=oldest;
	    r=false;
	}
    for(time_series::Index index=target; index<=newest; index++)
	{
	    v.push_back(observations_[index]);
	}
    return r;
}

TEMPLATE_FRONTEND
std::vector<Observation<NB_ACTUATORS,
			ROBOT_STATE,
			EXTENDED_STATE>>
FRONTEND::get_latest_observations(size_t nb_items)
{
    std::vector<Observation<NB_ACTUATORS,
			    ROBOT_STATE,
			    EXTENDED_STATE>> v;
    update_latest_observations(nb_items,v);
    return v;
			    
}

TEMPLATE_FRONTEND
void FRONTEND::add_command(int nb_actuator,
                           ROBOT_STATE target_state,
                           Iteration target_iteration,
                           Mode mode)
{
  Command<ROBOT_STATE> command(target_state,target_iteration,
			       nb_actuator,mode);
  buffer_commands_.append(command);
}

TEMPLATE_FRONTEND
void FRONTEND::add_command(int nb_actuator,
                           ROBOT_STATE target_state,
                           Speed speed,
                           Mode mode)
{
  Command<ROBOT_STATE> command(target_state,speed,
			       nb_actuator,mode);
  buffer_commands_.append(command);
}

TEMPLATE_FRONTEND
void FRONTEND::add_command(int nb_actuator,
                           ROBOT_STATE target_state,
                           Duration_us duration,
                           Mode mode)
{
  Command<ROBOT_STATE> command(target_state,duration,
			       nb_actuator,mode);
  buffer_commands_.append(command);
}

TEMPLATE_FRONTEND
void FRONTEND::add_command(int nb_actuator, ROBOT_STATE target_state, Mode mode)
{
  Command<ROBOT_STATE> command(target_state,
			       nb_actuator,mode);
  buffer_commands_.append(command);
}

TEMPLATE_FRONTEND
time_series::Index FRONTEND::last_index_read_by_backend()
{
  time_series::Index index;
  shared_memory::get<time_series::Index>(segment_id_,
					 "command_read",
					 index);
  return index;
					 
}

TEMPLATE_FRONTEND
void FRONTEND::share_commands(std::set<int>& command_ids, bool store)
{

  // checking we do have space in the shared memory for new commands
  if(!commands_.is_empty())
    {
      if(buffer_index_<0)
	{
	  buffer_index_=0;
	}
      time_series::Index oldest = commands_.oldest_timeindex(false);
      time_series::Index latest_read = last_index_read_by_backend();
      time_series::Index nb_slots = latest_read-oldest;
      time_series::Index nb_new_commands = latest_read - buffer_index_;
      if(nb_new_commands>nb_slots)
	{
	  throw std::runtime_error("shared memory for commands exchange full");    
	}
    }

  if(buffer_commands_.is_empty())
    {
      return;
    }
  
  // writing the commands into the shared time series
  time_series::Index last_index = buffer_commands_.newest_timeindex(false);

  if(last_index>=buffer_index_)
      {

	if (buffer_index_==-1)
	  {
	    buffer_index_=buffer_commands_.oldest_timeindex(false);
	  }

	for(time_series::Index index=buffer_index_; index<=last_index; index++)
	  {
	    Command<ROBOT_STATE> command = buffer_commands_[index];
	    if(store)
	      {
		command_ids.insert(command.get_id());
	      }
	    commands_.append(command);
	  }

	buffer_index_ = last_index+1;
      }

  // logging
  log(LogAction::FRONTEND_COMMUNICATE);

}

TEMPLATE_FRONTEND
void FRONTEND::wait_for_completion(std::set<int>& command_ids)
{
  while(completed_commands_.is_empty())
    {
      usleep(10);
    }
  time_series::Index oldest = completed_commands_.oldest_timeindex(false);
  if(last_completed_command_index_>=0 &&
     last_completed_command_index_<oldest)
    {
      throw std::runtime_error("shared memory for completed commands exchange full");    
    }
  time_series::Index latest = completed_commands_.newest_timeindex(false);
  if(last_completed_command_index_<0)
    {
      last_completed_command_index_ = completed_commands_.oldest_timeindex(false);
    }
  time_series::Index obs_index;
  while(true)
    {
      obs_index = observations_.newest_timeindex();
      for(time_series::Index index = last_completed_command_index_;
	  index<=latest;index++)
	{
	  time_series::Index command_id = completed_commands_[index];
	  command_ids.erase(command_id);
	  if (command_ids.empty())
	    {
	      return;
	    }
	  latest=completed_commands_.newest_timeindex(false);
	}
      observations_.wait_for_timeindex(obs_index+1);
    }
}

TEMPLATE_FRONTEND
Observation<NB_ACTUATORS, ROBOT_STATE, EXTENDED_STATE> FRONTEND::pulse(
    Iteration iteration)
{
  share_commands(sent_command_ids_,false);
  observations_.wait_for_timeindex(iteration.value);
  return observations_[iteration.value];
}

TEMPLATE_FRONTEND
Observation<NB_ACTUATORS, ROBOT_STATE, EXTENDED_STATE> FRONTEND::pulse()
{
  share_commands(sent_command_ids_,false);
  if(observations_.is_empty())
    {
      return Observation<NB_ACTUATORS, ROBOT_STATE, EXTENDED_STATE>();
    }
  return observations_.newest_element();
}


TEMPLATE_FRONTEND
Observation<NB_ACTUATORS, ROBOT_STATE, EXTENDED_STATE>
FRONTEND::pulse_and_wait()
{
  sent_command_ids_.clear();
  share_commands(sent_command_ids_,true);
  wait_for_completion(sent_command_ids_);
  return observations_.newest_element();
}


TEMPLATE_FRONTEND
Observation<NB_ACTUATORS, ROBOT_STATE, EXTENDED_STATE> FRONTEND::burst(
    int nb_iterations)
{
  share_commands(sent_command_ids_,false);
  internal::set_bursting(segment_id_, nb_iterations);
  if (leader_ == nullptr)
    {
      leader_.reset(
		    new synchronizer::Leader(segment_id_ + "_synchronizer", true));
    }
  leader_->pulse();
  if(observations_.is_empty())
    {
      return Observation<NB_ACTUATORS, ROBOT_STATE, EXTENDED_STATE>();
    }
  return observations_.newest_element();
}

TEMPLATE_FRONTEND
void FRONTEND::final_burst()
{
    leader_->stop_sync();
}


TEMPLATE_FRONTEND
Observation<NB_ACTUATORS, ROBOT_STATE, EXTENDED_STATE> FRONTEND::read()
{
  Observation<NB_ACTUATORS, ROBOT_STATE, EXTENDED_STATE> observation;
  if(observations_.is_empty())
    {
      return observation;
    }
  observation = observations_.newest_element();
  log(LogAction::FRONTEND_READ);
  return observation;
}
