#pragma once

#include "o80/command_types.hpp"
#include "o80/sensor_state.hpp"
#include "o80/time.hpp"

namespace o80
{
class VoidState : public SensorState
{
public:
    VoidState();

    std::string to_string() const;

    void get() const
    {
    }
    void set() const
    {
    }

    template <class Archive>
    void serialize(Archive &archive)
    {
    }
};
}  // namespace o80
