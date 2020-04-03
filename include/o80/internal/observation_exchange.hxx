// Copyright (c) 2019 Max Planck Gesellschaft
// Author : Vincent Berenz

#define TEMPLATE_OE \
    template <int NB_ACTUATORS, class ROBOT_STATE, class EXTENDED_STATE>

#define OBSERVATIONEXCHANGE \
    ObservationExchange<NB_ACTUATORS, ROBOT_STATE, EXTENDED_STATE>

TEMPLATE_OE
OBSERVATIONEXCHANGE::ObservationExchange(std::string segment_id,
                                         std::string object_id,
					 size_t history_size,
					 bool leader)
    : segment_id_(segment_id), object_id_(object_id),
      segment_id_history_(segment_id+"_history"),
      history_(segment_id_history_,history_size,leader)
{
}

TEMPLATE_OE
time_series::MultiprocessTimeSeries<OBSERVATION>& OBSERVATIONEXCHANGE::get_history()
{
    return history_;
}

TEMPLATE_OE
void OBSERVATIONEXCHANGE::write(
    const Observation<NB_ACTUATORS, ROBOT_STATE, EXTENDED_STATE>& observation)
{
    mutex_.lock();
    serialized_ = serializer_.serialize(observation);
    shared_memory::set(segment_id_, object_id_, serialized_);
    mutex_.unlock();
    history_.append(observation);
}


TEMPLATE_OE
bool OBSERVATIONEXCHANGE::read(
    Observation<NB_ACTUATORS, ROBOT_STATE, EXTENDED_STATE>& observation)
{
    mutex_.lock();
    shared_memory::get(segment_id_, object_id_, serialized_);
    if (serialized_.size() == 0)
    {
        mutex_.unlock();
        return false;
    }
    serializer_.deserialize(serialized_, observation);
    mutex_.unlock();
    return true;
}
