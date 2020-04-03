// Copyright (c) 2019 Max Planck Gesellschaft
// Author : Vincent Berenz

#pragma once

#include <mutex>
#include "o80/observation.hpp"
#include "shared_memory/serializer.hpp"
#include "shared_memory/shared_memory.hpp"
#include "time_series/multiprocess_time_series.hpp"

namespace o80
{
template <int NB_ACTUATORS,
          class ROBOT_STATE,
          class EXTENDED_STATE = EmptyExtendedState>
class ObservationExchange
{

public:
    
    typedef time_series::MultiprocessTimeSeries<Observation<NB_ACTUATORS,
							    ROBOT_STATE,
							    EXTENDED_STATE>> History;
    
public:
    ObservationExchange(std::string segment_id,
			std::string object_id,
			size_t history_size=5000,
			bool leader=true);

    void write(const Observation<NB_ACTUATORS, ROBOT_STATE, EXTENDED_STATE>& observation);
    bool read(
        Observation<NB_ACTUATORS, ROBOT_STATE, EXTENDED_STATE>& observation);

    History& get_history(); 
    
private:
    std::mutex mutex_;
    std::string serialized_;
    std::string segment_id_;
    std::string segment_id_history_;
    std::string object_id_;

    History history_;
    
    shared_memory::Serializer<
        Observation<NB_ACTUATORS, ROBOT_STATE, EXTENDED_STATE>>
        serializer_;
};

#include "observation_exchange.hxx"
}
