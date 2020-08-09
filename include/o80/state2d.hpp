#pragma once

#include "o80/statexd.hpp"

namespace o80
{
/* ! StateXd with 2 double attributes */
class State2d : public StateXd<State2d, double, double>
{
public:
    State2d(double x, double y) : StateXd<State2d, double, double>(x, y)
    {
    }
    State2d() : StateXd<State2d, double, double>()
    {
    }
};
}  // namespace o80
