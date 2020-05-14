#pragma once
#include <iostream>
#include <fstream>
#include <string>
#include <stdexcept>
#include <tuple>
#include "shared_memory/serializer.hpp"
#include "time_series/multiprocess_time_series.hpp"

namespace o80
{

  enum LogAction
    {
      FRONTEND_WAIT_START,
      FRONTEND_WAIT_END,
      FRONTEND_COMMUNICATE,
      FRONTEND_READ,
      FRONTEND_COMPLETION_WAIT_START,
      FRONTEND_COMPLETION_WAIT_END,
      BACKEND_READ,
      BACKEND_WRITE_REAPPLY,
      BACKEND_WRITE_NEW
    };
  
  class LogEntry
  {
  public:

    LogEntry(){}
    LogEntry(std::string segment_id, LogAction action);
      
    template <class Archive>
    void serialize(Archive& archive)
    {
      archive(segment_id,
	      action);

    }
    
    static constexpr int segment_id_size{30};
    char segment_id[segment_id_size];
    LogAction action;
    
  };

  
  class Logger
  {
  public:
    
    Logger(int queue_size,
	   std::string segment_id,
	   bool leader);
    void log(std::string user_segment_id, LogAction action);
    
    std::size_t length();
    
    std::tuple<time_series::Timestamp,LogEntry> get(time_series::Index index,
						    time_series::Index start_index=-1);
    
    void save(std::string path);

  private:

    std::string segment_id_;
    time_series::MultiprocessTimeSeries<LogEntry> logs_;
    
  };

}
