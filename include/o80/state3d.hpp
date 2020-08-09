#pragma once

#include "o80/statexd.hpp"

namespace o80
{
/* ! StateXd with 3 double attributes */
class State3d : public StateXd<State3d, double, double, double>
{
public:
    State3d(double x, double y, double z)
        : StateXd<State3d, double, double, double>(x, y, z)
    {
    }
    State3d() : StateXd<State3d, double, double, double>()
    {
    }
};
}  // namespace o80
