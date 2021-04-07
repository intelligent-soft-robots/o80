#include "o80/bool_state.hpp"

namespace o80
{
BoolState::BoolState() : SensorState{}, status_{false}
{
}

BoolState::BoolState(bool status) : SensorState{}, status_{status}
{
}

void BoolState::set(bool value)
{
    status_ = value;
}

bool BoolState::get() const
{
    return status_;
}

std::string BoolState::to_string() const
{
    return std::to_string(status_);
}

}  // namespace o80
