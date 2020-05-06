#include "o80/burster.hpp"

namespace o80
{
Burster::Burster(std::string segment_id)
    : segment_id_(segment_id),
      nb_bursts_(0),
      nb_iterated_(-1),
      running_(true),
      follower_(nullptr)
{
    Burster::turn_on(segment_id);
    reset_nb_bursts();
}

Burster::~Burster()
{
    clear_memory(segment_id_);
}

void Burster::clear_memory(std::string segment_id)
{
    shared_memory::clear_shared_memory(segment_id + "_synchronizer");
    shared_memory::clear_shared_memory(segment_id);
}

void Burster::turn_on(std::string segment_id)
{
    shared_memory::set<bool>(segment_id, "should_burst", true);
}

void Burster::turn_off(std::string segment_id)
{
    shared_memory::set<bool>(segment_id, "should_burst", false);
}

int Burster::get_nb_bursts() const
{
    int v;
    shared_memory::get(segment_id_, "bursting", v);
    return v;
}

void Burster::reset_nb_bursts()
{
    shared_memory::set<double>(segment_id_, "bursting", 0.0);
}

bool Burster::should_run() const
{
    bool sr;
    shared_memory::get<bool>(segment_id_, "should_burst", sr);
    return sr;
}

bool Burster::pulse()
{
    running_ = should_run();

    if (!running_)
    {
        return false;
    }

    if (follower_ == nullptr)
    {
        follower_.reset(
            new synchronizer::Follower(segment_id_ + "_synchronizer", 1, true));
    }

    if (nb_iterated_ < 0)
    {
        follower_->pulse();
        nb_bursts_ = get_nb_bursts();
        if (nb_bursts_ >= 0)
        {
            nb_iterated_ = 1;
	    if(nb_iterated_ >= nb_bursts_)
		{
		    nb_iterated_ = -1;
		    reset_nb_bursts();
		}
        }
    }
    else
    {
        if ((nb_iterated_ + 1) < nb_bursts_)
        {
            nb_iterated_++;
        }
        else
        {
            nb_iterated_ = -1;
            reset_nb_bursts();
        }
    }

    return true;
}
}
