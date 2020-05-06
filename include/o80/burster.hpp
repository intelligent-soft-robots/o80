#pragma once

#include <memory>

#include "shared_memory/shared_memory.hpp"
#include "synchronizer/follower.hpp"

namespace o80
{
// for synchronization when
// standalone is not used
class Burster
{
public:
    Burster(std::string segment_id);
    ~Burster();

    bool pulse();

public:
    static void clear_memory(std::string segment_id);
    static void turn_on(std::string segment_id);
    static void turn_off(std::string segment_id);

private:
    int get_nb_bursts() const;
    void reset_nb_bursts();
    bool should_run() const;

private:
    std::string segment_id_;
    int nb_bursts_;
    int nb_iterated_;
    bool running_;

    std::shared_ptr<synchronizer::Follower> follower_;
};
}
