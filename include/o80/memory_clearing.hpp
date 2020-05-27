#pragma once


#include "shared_memory/shared_memory.hpp"
#include "shared_memory/mutex.hpp"

namespace o80
{

    void clear_shared_memory(std::string segment_id)
    {
	shared_memory::clear_shared_memory(segment_id);
	shared_memory::clear_shared_memory(segment_id+std::string("_commands"));
	shared_memory::clear_shared_memory(segment_id+std::string("_observations"));
	shared_memory::clear_shared_memory(segment_id+std::string("_completed"));
	shared_memory::clear_shared_memory(segment_id +
					   std::string("_synchronizer"));
	shared_memory::clear_shared_memory(segment_id +
					   std::string("_synchronizer_follower"));
	shared_memory::clear_shared_memory(segment_id +
					   std::string("_synchronizer_leader"));
	// mutex cleaned on destruction
	shared_memory::Mutex m1(segment_id + std::string("_locker"), true);
	shared_memory::Mutex m2(
				std::string("completed_") +
				segment_id + std::string("_locker"), true);
    }

}
