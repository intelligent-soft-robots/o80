#pragma once

#include "o80/burster.hpp"
#include "shared_memory/shared_memory.hpp"
#include "time_series/multiprocess_time_series.hpp"

namespace o80
{
/*! clear all the shared memory segments corresponding to
 *  the specified segment id. The destructor of BackEnd calls
 *  this function, but user code may call it in case a program
 *  has been abruptly terminated (i.e. the BackEnd 's destructor has not
 *  been called). Starting a BackEnd using a segment id that
 *  has not been properly cleared will result in the program hanging.
 */
void clear_shared_memory(std::string segment_id)
{
    time_series::clear_memory(segment_id + "_commands");
    time_series::clear_memory(segment_id + "_observations");
    time_series::clear_memory(segment_id + "_completed");
    time_series::clear_memory(segment_id + "_waiting_for_completion");
    time_series::clear_memory(segment_id + "_completion_reported");
    time_series::clear_memory(segment_id + "_received");
    time_series::clear_memory(segment_id + "_starting");
    shared_memory::clear_shared_memory(segment_id +
                                       std::string("_commands"));
    shared_memory::clear_shared_memory(segment_id +
                                       std::string("_observations"));
    shared_memory::clear_shared_memory(segment_id +
                                       std::string("_completed"));
    shared_memory::clear_shared_memory(segment_id +
                                       std::string("_waiting_for_completion"));
    shared_memory::clear_shared_memory(segment_id +
                                       std::string("_completion_reported"));
    shared_memory::clear_shared_memory(segment_id +
                                       std::string("_received"));
    shared_memory::clear_shared_memory(segment_id +
                                       std::string("_starting"));
    Burster::clear_memory(segment_id);
    shared_memory::clear_shared_memory(segment_id +
                                       std::string("_synchronizer"));
    shared_memory::clear_shared_memory(segment_id +
                                       std::string("_synchronizer_follower"));
    shared_memory::clear_shared_memory(segment_id +
                                       std::string("_synchronizer_leader"));
    shared_memory::clear_shared_memory(segment_id);
}
}  // namespace o80
