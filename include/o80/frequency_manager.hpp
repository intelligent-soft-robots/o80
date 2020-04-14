#pragma once

#include <chrono>
#include <time.h>
#include "o80/typedefs.hpp"

namespace o80
{

    class FrequencyManager
    {

    public:

	FrequencyManager(double frequency);
	void wait();
	
    private:

	timespec req_;
	TimePoint previous_time_;
	Nanoseconds period_;

    };
    

}
