#pragma once

#include <string>
#include "o80/sensor_state.hpp"

namespace o80
{
/*! A State encapsulating a boolean
 *
 */
class BoolState : public SensorState
{
public:
    BoolState();
    BoolState(bool value);

    void set(bool value);
    bool get() const;
    std::string to_string() const;

    template <class Archive>
    void serialize(Archive &archive)
    {
        archive(status_);
    }

private:
    bool status_;
};
}  // namespace o80
