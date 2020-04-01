// Copyright (c) 2019 Max Planck Gesellschaft
// Author : Vincent Berenz

#pragma once

#include <mutex>
#include "o80/observation.hpp"
#include "shared_memory/serializer.hpp"
#include "shared_memory/shared_memory.hpp"

namespace o80
{
template <int NB_ACTUATORS,
          class ROBOT_STATE,
          class EXTENDED_STATE = EmptyExtendedState>
class ObservationExchange
{
public:
    ObservationExchange(std::string segment_id, std::string object_id);

    void write(const Observation<NB_ACTUATORS, ROBOT_STATE, EXTENDED_STATE>&
                   observation);
    bool read(
        Observation<NB_ACTUATORS, ROBOT_STATE, EXTENDED_STATE>& observation);

private:
    std::mutex mutex_;
    std::string serialized_;
    std::string segment_id_;
    std::string object_id_;

    shared_memory::Serializer<
        Observation<NB_ACTUATORS, ROBOT_STATE, EXTENDED_STATE>>
        serializer_;
};

#include "observation_exchange.hxx"
}
