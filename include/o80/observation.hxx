// Copyright (c) 2019 Max Planck Gesellschaft
// Author : Vincent Berenz

#define TEMPLATE_OBSERVATION \
    template <int NB_ACTUATORS, class ROBOT_STATE, class EXTENDED_STATE>

#define OBSERVATION Observation<NB_ACTUATORS, ROBOT_STATE, EXTENDED_STATE>

TEMPLATE_OBSERVATION
OBSERVATION::Observation()
    : control_iteration_(-1),
      sensor_iteration_(-1),
      stamp_(-1),
      observed_frequency_(-1)
{
}

TEMPLATE_OBSERVATION
void OBSERVATION::copy(const OBSERVATION& from, bool full)
{
    if (full)
    {
        observed_states_ = from.observed_states_;
        desired_states_ = from.desired_states_;
        extended_state_ = from.extended_state_;
        stamp_ = from.stamp_;
    }
    control_iteration_ = from.control_iteration_;
    sensor_iteration_ = from.sensor_iteration_;
    observed_frequency_ = from.observed_frequency_;
}

TEMPLATE_OBSERVATION
OBSERVATION::Observation(const OBSERVATION& other)
{
    copy(other, true);
}

TEMPLATE_OBSERVATION
OBSERVATION::Observation(OBSERVATION&& other) noexcept
    : observed_states_(std::move(other.observed_states_)),
      desired_states_(std::move(other.desired_states_)),
      extended_state_(std::move(other.extended_state_))
{
    copy(other, false);
}

TEMPLATE_OBSERVATION
OBSERVATION& OBSERVATION::operator=(const OBSERVATION& other)
{
    copy(other, true);
    return *this;
}

TEMPLATE_OBSERVATION
OBSERVATION& OBSERVATION::operator=(OBSERVATION&& other) noexcept
{
    copy(other, false);
    observed_states_ = std::move(other.observed_states_);
    desired_states_ = std::move(other.desired_states_);
    extended_state_ = std::move(other.extended_state_);
    return *this;
}

TEMPLATE_OBSERVATION
OBSERVATION::Observation(States<NB_ACTUATORS, ROBOT_STATE> observed_states,
                         States<NB_ACTUATORS, ROBOT_STATE> desired_states,
                         long int stamp,
                         long int iteration,
                         double frequency)
    : control_iteration_(iteration),
      sensor_iteration_(iteration),
      stamp_(stamp),
      observed_states_(observed_states),
      desired_states_(desired_states),
      observed_frequency_(frequency)
{
}

TEMPLATE_OBSERVATION
OBSERVATION::Observation(States<NB_ACTUATORS, ROBOT_STATE> observed_states,
                         States<NB_ACTUATORS, ROBOT_STATE> desired_states,
                         long int stamp,
                         long int control_iteration,
                         long int sensor_iteration,
                         double frequency)
    : control_iteration_(control_iteration),
      sensor_iteration_(sensor_iteration),
      stamp_(stamp),
      observed_states_(observed_states),
      desired_states_(desired_states),
      observed_frequency_(frequency)
{
}

TEMPLATE_OBSERVATION
OBSERVATION::Observation(States<NB_ACTUATORS, ROBOT_STATE> observed_states,
                         States<NB_ACTUATORS, ROBOT_STATE> desired_states,
                         EXTENDED_STATE extended_state,
                         long int stamp,
                         long int iteration,
                         double frequency)
    : control_iteration_(iteration),
      sensor_iteration_(iteration),
      extended_state_(extended_state),
      stamp_(stamp),
      observed_states_(observed_states),
      desired_states_(desired_states),
      observed_frequency_(frequency)
{
}

TEMPLATE_OBSERVATION
OBSERVATION::Observation(States<NB_ACTUATORS, ROBOT_STATE> observed_states,
                         States<NB_ACTUATORS, ROBOT_STATE> desired_states,
                         EXTENDED_STATE extended_state,
                         long int stamp,
                         long int control_iteration,
                         long int sensor_iteration,
                         double frequency)
    : control_iteration_(control_iteration),
      sensor_iteration_(sensor_iteration),
      extended_state_(extended_state),
      stamp_(stamp),
      observed_states_(observed_states),
      desired_states_(desired_states),
      observed_frequency_(frequency)
{
}

TEMPLATE_OBSERVATION
const States<NB_ACTUATORS, ROBOT_STATE>& OBSERVATION::get_observed_states()
    const
{
    return observed_states_;
}

TEMPLATE_OBSERVATION
const States<NB_ACTUATORS, ROBOT_STATE>& OBSERVATION::get_desired_states() const
{
    return desired_states_;
}

TEMPLATE_OBSERVATION
const EXTENDED_STATE& OBSERVATION::get_extended_state() const
{
    return extended_state_;
}

TEMPLATE_OBSERVATION
long int OBSERVATION::get_control_iteration() const
{
    return control_iteration_;
}

TEMPLATE_OBSERVATION
long int OBSERVATION::get_sensor_iteration() const
{
    return sensor_iteration_;
}

TEMPLATE_OBSERVATION
long int OBSERVATION::get_iteration() const
{
    return control_iteration_;
}

TEMPLATE_OBSERVATION
double OBSERVATION::get_frequency() const
{
    return observed_frequency_;
}

TEMPLATE_OBSERVATION
long int OBSERVATION::get_time_stamp() const
{
    return stamp_;
}


TEMPLATE_OBSERVATION
std::string OBSERVATION::to_string() const
{
    std::stringstream ss;
    ss << "iteration: " << control_iteration_
       << " (frequency: " << observed_frequency_ << ")\n";
    ss << "current states:\n";
    for (int i = 0; i < NB_ACTUATORS; i++)
    {
        ss << "\t" << i << "\t" << observed_states_.get(i).to_string() << "\n";
    }
    ss << "desired states:\n";
    for (int i = 0; i < NB_ACTUATORS; i++)
    {
        ss << "\t" << i << "\t" << desired_states_.get(i).to_string() << "\n";
    }
    return ss.str();
}
