#include "o80/memory_clearing.hpp"

namespace o80
{

  void clear_shared_memory(std::string segment_id)
{
    time_series::clear_memory(segment_id + "_commands");
    time_series::clear_memory(segment_id + "_observations");
    time_series::clear_memory(segment_id + "_completed");
    time_series::clear_memory(segment_id + "_waiting_for_completion");
    time_series::clear_memory(segment_id + "_completion_reported");
    time_series::clear_memory(segment_id + "_received");
    time_series::clear_memory(segment_id + "_starting");
    shared_memory::clear_shared_memory(segment_id + std::string("_commands"));
    shared_memory::clear_shared_memory(segment_id +
                                       std::string("_observations"));
    shared_memory::clear_shared_memory(segment_id + std::string("_completed"));
    shared_memory::clear_shared_memory(segment_id +
                                       std::string("_waiting_for_completion"));
    shared_memory::clear_shared_memory(segment_id +
                                       std::string("_completion_reported"));
    shared_memory::clear_shared_memory(segment_id + std::string("_received"));
    shared_memory::clear_shared_memory(segment_id + std::string("_starting"));
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
