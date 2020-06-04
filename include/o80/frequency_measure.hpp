#pragma once

#include <chrono>
#include "o80/typedefs.hpp"

namespace o80
{
class FrequencyMeasure
{
public:
    FrequencyMeasure();
    double tick();

private:
    TimePoint previous_time_;
};
}
