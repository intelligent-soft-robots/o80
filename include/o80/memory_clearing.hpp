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
  void clear_shared_memory(std::string segment_id);
}  // namespace o80
