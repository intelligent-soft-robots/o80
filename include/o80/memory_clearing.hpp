#pragma once

#include "shared_memory/shared_memory.hpp"
#include "time_series/multiprocess_time_series.hpp"
#include "o80/burster.hpp"

namespace o80
{
void clear_shared_memory(std::string segment_id)
{

  time_series::clear_memory(segment_id + "_commands");
  time_series::clear_memory(segment_id + "_observations");
  time_series::clear_memory(segment_id + "_completed");
  Burster::clear_memory(segment_id);
  shared_memory::clear_shared_memory(segment_id +
				     std::string("_synchronizer"));
  shared_memory::clear_shared_memory(segment_id +
				     std::string("_synchronizer_follower"));
  shared_memory::clear_shared_memory(segment_id +
				     std::string("_synchronizer_leader"));
    shared_memory::clear_shared_memory(segment_id);
}
}
