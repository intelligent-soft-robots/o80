

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
  template <class ROBOT_STATE>
  THREAD_FUNCTION_RETURN_TYPE run_received_helper(void *arg)
  {
    ((Introspector<ROBOT_STATE>*)arg)->run_received();
    return THREAD_FUNCTION_RETURN_VALUE;
  }
  template <class ROBOT_STATE>
  THREAD_FUNCTION_RETURN_TYPE run_starting_helper(void *arg)
  {
    ((Introspector<ROBOT_STATE>*)arg)->run_starting();
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
								      "_completion_reported")},
    received_{CompletedCommandsTimeSeries::create_follower_ptr(
							       segment_id +
							       "_received")},
    starting_{CompletedCommandsTimeSeries::create_follower_ptr(
							       segment_id +
							       "_starting")}
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
  received_thread_.create_realtime_thread(internal::run_received_helper<ROBOT_STATE>,
					  (void*)this);
  starting_thread_.create_realtime_thread(internal::run_starting_helper<ROBOT_STATE>,
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

template<class T>
std::string to_string(const T& element)
{
  if constexpr (std::is_fundamental<T>::value)
    {
      return std::to_string(element);
    }
  else
    {
      return element.to_string();
    }
}

template<class ROBOT_STATE>
template<class T>
void Introspector<ROBOT_STATE>::run(std::shared_ptr<T> time_series, std::string prefix)
{
  time_series::Index index{time_series::EMPTY};
  while (true)
    {
      index = time_series->newest_timeindex(false);
      if(index!=time_series::EMPTY)
	{
	  break;
	}
      if(!running_)
	{
	  return;
	}
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  while(running_)
    {
      auto element = (*time_series)[index++];
      std::string str = to_string(element);
      {
	std::lock_guard<std::mutex> guard(mutex_);
	std::cout << prefix << str << std::endl;
      }
      while(running_)
	{
	  bool received = time_series->wait_for_timeindex(index,0.0001);
	  if (received)
	    {
	      break;
	    }
	}
    }
}

template<class ROBOT_STATE>
void Introspector<ROBOT_STATE>::run_commands()
{
  run(commands_,std::string("~frontend~ shares: "));
}


template<class ROBOT_STATE>
void Introspector<ROBOT_STATE>::run_completed_commands()
{
  run(completed_commands_,
       "\t\t\t\t\t\t\t\t\t*backend* reports completion of: ");
}

template<class ROBOT_STATE>
void Introspector<ROBOT_STATE>::run_waiting_for_completion()
{
  run(waiting_for_completion_,
      "\t~frontend~ waiting for completion of: ");
}

template<class ROBOT_STATE>
void Introspector<ROBOT_STATE>::run_completion_reported()
{
  run(completion_reported_,
      "\t\t~frontend~ confirms completion of: ");
}

template<class ROBOT_STATE>
void Introspector<ROBOT_STATE>::run_received()
{
  run(received_,
      "\t\t\t\t\t\t\t*backend* received: ");
}

template<class ROBOT_STATE>
void Introspector<ROBOT_STATE>::run_starting()
{
  run(received_,
      "\t\t\t\t\t\t\t\t*backend* starting: ");
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
