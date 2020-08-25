// Copyright (c) 2019 Max Planck Gesellschaft
// Author : Vincent Berenz

template <int NB_ACTUATORS, class STATE>
void States<NB_ACTUATORS, STATE>::set(int actuator, STATE state)
{
    if (actuator < 0 || actuator >= NB_ACTUATORS)
    {
        throw std::runtime_error("invalid actuator index");
    }
    values[actuator] = state;
}

template <int NB_ACTUATORS, class STATE>
const STATE& States<NB_ACTUATORS, STATE>::get(int actuator) const
{
    if (actuator < 0 || actuator >= NB_ACTUATORS)
    {
        throw std::runtime_error("invalid index");
    }
    return values[actuator];
}

