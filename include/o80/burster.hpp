#pragma once

#include <memory>

#include "shared_memory/shared_memory.hpp"
#include "synchronizer/follower.hpp"
#include "synchronizer/leader.hpp"

namespace o80
{
/*! Class managing the bursting mode of BackEnd
 *  and Standalone. Expected usage:
 *  \code{.cpp}
 *  Burster burster(segment_id);
 *  while(true) {
 *      std::cout << "iterate!" << std::endl;
 *      burster.pulse()
 *  }
 *  \endcode
 *  The code above will wait until the method "burst"
 *  of a related FrontEnd is called, which triggers
 *  one iteration to occur.
 */
class Burster
{
public:
    Burster(std::string segment_id);
    ~Burster();

    bool pulse();

public:
    /*! If an instance of Burster has not been cleanly exited
        (i.e. destructor not called), wipe the related shared memory
      (hanging at startup may otherwise occuring)*/
    static void clear_memory(std::string segment_id);
    /*! Turn on the bursting mode of the burster, i.e.
     *  it will iterate only when receiving a signal
     *  from the frontend.
     */
    static void turn_on(std::string segment_id);
    /*! Disable the bursting mode, i.e. the pulse
     *  method will always return immediately.
     */
    static void turn_off(std::string segment_id);

private:
    long int get_nb_bursts() const;
    void reset_nb_bursts();
    bool should_run() const;

private:
    std::string segment_id_;
    long int nb_bursts_;
    long int nb_iterated_;
    bool running_;

    std::shared_ptr<synchronizer::Follower> follower_;
};

class BursterClient
{
public:
  typedef std::shared_ptr<synchronizer::Leader> LeaderPtr;

  BursterClient(std::string segment_id)
    : leader_{segment_id+"_synchronizer",true},
      segment_id_{segment_id}
  {
    set_bursting(1);
  }

  void burst(int nb_iterations)
  {
    set_bursting(nb_iterations);
    leader_.pulse();
  }

  void final_burst()
  {
    leader_.stop_sync();
  }
  
private:
  void set_bursting(int nb_iterations)
  {
    shared_memory::set<long int>(segment_id_, "bursting", nb_iterations);
    shared_memory::set<long int>(segment_id_, "bursting_sync", nb_iterations);
  }
private:
  synchronizer::Leader leader_;
  std::string segment_id_;



  
};

  
}  // namespace o80
