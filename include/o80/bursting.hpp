#pragma once

#include "shared_memory/shared_memory.hpp"
#include "synchronizer/follower.hpp"

namespace o80
{
class Burster
{
public:
    Burster(std::segment_id);
    ~Burster();

    void pulse();

public:
    static void clear_memory(std::segment_id);

private:
    void update_nb_bursts();
    void reset_nb_bursts();

private:
    std::string segment_id_;
    std::string object_id_;
    uint nb_bursts_;
    uint nb_iterated_;

    synchronizer::Follower follower_;
};
}  // namespace o80
