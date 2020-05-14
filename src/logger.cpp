#include "o80/logger.hpp"

namespace o80
{

    LogEntry::LogEntry(std::string sid, LogAction a)
      : action(a)
    {
      if(sid.length()>segment_id_size)
	{
	  std::string error;
	  error = "O80 logger: the maximum length of a segment_id is ";
	  error += std::to_string(segment_id_size);
	  error += " ("+sid+")";
	  throw std::runtime_error(error);
	}
      std::size_t length = sid.copy(segment_id,sid.length());
      segment_id[length]='\0';
    }

  Logger::Logger(int queue_size,
		 std::string sid,
		 bool leader)
    : segment_id_(sid),
      logs_(segment_id_,queue_size,leader) {}
  
  void Logger::log(std::string user_segment_id, LogAction action)
  {
    logs_.append(LogEntry(user_segment_id,action));
  }
  
  std::size_t Logger::length()
  {
    return logs_.length();
  }
  
  std::tuple<time_series::Timestamp,LogEntry> Logger::get(time_series::Index index,
							  time_series::Index start_index)
  {
    if(start_index<0)
      {
	start_index = logs_.oldest_timeindex();
      }
    LogEntry entry = logs_[start_index+index];
    time_series::Timestamp stamp = logs_.timestamp_ms(start_index+index);
    return std::make_tuple(stamp,entry);
	
  }
  
  void Logger::save(std::string path)
  {
      std::cout << "O80 logger, saving in: " << path << "\n";
    std::ofstream f;
    f.open(path);
    std::size_t l = length();
    time_series::Index start_index = logs_.oldest_timeindex();
    time_series::Index last_index = logs_.newest_timeindex(false);
    time_series::Timestamp baseline = logs_.timestamp_ms(last_index);
    for(long int i=0;i<l;i++)
      {
	std::tuple<time_series::Timestamp,LogEntry> stamp_entry = get(i,start_index);
	f << std::get<1>(stamp_entry).segment_id << "\t" << std::get<1>(stamp_entry).action
	  << "\t" << baseline - std::get<0>(stamp_entry) << "\n";
      }
    f.close();
    std::cout << "saved ! " << path << " \n";
  }

  
  
}
