

namespace internal
{
  template <class ROBOT_STATE>
  THREAD_FUNCTION_RETURN_TYPE run_commands_helper(void *arg)
  {
    ((Introspector<ROBOT_STATE>*)arg)->run_commands();
    return THREAD_FUNCTION_RETURN_VALUE;
  }
  template <class ROBOT_STATE>
  THREAD_FUNCTION_RETURN_TYPE run_completed_commands_helper(void *arg)
  {
    ((Introspector<ROBOT_STATE>*)arg)->run_completed_commands();
    return THREAD_FUNCTION_RETURN_VALUE;
  }
  template <class ROBOT_STATE>
  THREAD_FUNCTION_RETURN_TYPE run_waiting_for_completion_helper(void *arg)
  {
    ((Introspector<ROBOT_STATE>*)arg)->run_waiting_for_completion();
    return THREAD_FUNCTION_RETURN_VALUE;
  }
  template <class ROBOT_STATE>
  THREAD_FUNCTION_RETURN_TYPE run_completion_reported_helper(void *arg)
  {
    ((Introspector<ROBOT_STATE>*)arg)->run_completion_reported();
    return THREAD_FUNCTION_RETURN_VALUE;
  }
}


template<class ROBOT_STATE>
Introspector<ROBOT_STATE>::Introspector(std::string segment_id)
  : running_{false},
    commands_{CommandsTimeSeries::create_follower_ptr(segment_id + "_commands")},
    completed_commands_{CompletedCommandsTimeSeries::create_follower_ptr(
								     segment_id + "_completed")},
    waiting_for_completion_{CompletedCommandsTimeSeries::create_follower_ptr(
									 segment_id +
									 "_waiting_for_completion")},
    completion_reported_{CompletedCommandsTimeSeries::create_follower_ptr(
								      segment_id +
								      "_completion_reported")}
{}

template<class ROBOT_STATE>
void Introspector<ROBOT_STATE>::start()
{
  running_=true;
  commands_thread_.create_realtime_thread(internal::run_commands_helper<ROBOT_STATE>,
					  (void*)this);
  completed_commands_thread_.create_realtime_thread(internal::run_completed_commands_helper<ROBOT_STATE>,
						    (void*)this);
  waiting_for_completion_thread_.create_realtime_thread(internal::run_waiting_for_completion_helper<ROBOT_STATE>,
							(void*)this);
  completion_reported_thread_.create_realtime_thread(internal::run_completion_reported_helper<ROBOT_STATE>,
						     (void*)this);
}

template<class ROBOT_STATE>
void Introspector<ROBOT_STATE>::stop()
{
  running_=false;
  commands_thread_.join();
  completed_commands_thread_.join();
  waiting_for_completion_thread_.join();
  completion_reported_thread_.join();
}

template<class ROBOT_STATE>
void Introspector<ROBOT_STATE>::run_commands()
{
  while(running_)
    {
      Command<ROBOT_STATE> command = commands_->newest_element();
      std::string str = command.to_string();
      {
	std::lock_guard<std::mutex> guard(mutex_);
	std::cout << "frontend shares command: " << str << std::endl;
      }
    }
}


template<class ROBOT_STATE>
void Introspector<ROBOT_STATE>::run_completed_commands()
{
  run_command_ids(completed_commands_,
		  "\t\t\t\t\tbackend reports completion of: ");
}

template<class ROBOT_STATE>
void Introspector<ROBOT_STATE>::run_waiting_for_completion()
{
  run_command_ids(waiting_for_completion_,
		  "\tfrontend waiting for completion of: ");
}

template<class ROBOT_STATE>
void Introspector<ROBOT_STATE>::run_completion_reported()
{
  run_command_ids(completion_reported_,
		  "\t\tfrontend confirms completion of: ");
}

template<class ROBOT_STATE>
void Introspector<ROBOT_STATE>::run_command_ids(std::shared_ptr<CompletedCommandsTimeSeries> time_series,
						std::string prefix)
{
  while(running_)
    {
      int command_id = time_series->newest_element();
      {
	std::lock_guard<std::mutex> guard(mutex_);
	std::cout << prefix << command_id << std::endl;
      }
    }
}

template<class ROBOT_STATE>
std::shared_ptr<Introspector<ROBOT_STATE>> Introspector<ROBOT_STATE>::instance_=nullptr;

template<class ROBOT_STATE>
bool Introspector<ROBOT_STATE>::start_running(std::string segment_id)
{
  if(Introspector<ROBOT_STATE>::instance_)
    {
      return false;
    }
  Introspector<ROBOT_STATE>::instance_ = std::make_shared<Introspector<ROBOT_STATE>>(segment_id);
  Introspector<ROBOT_STATE>::instance_->start();
  return true;
}

template<class ROBOT_STATE>
bool Introspector<ROBOT_STATE>::stop_running()
{
  if(!Introspector<ROBOT_STATE>::instance_)
    {
      return false;
    }
  Introspector<ROBOT_STATE>::instance_->stop();
  return true;
}
