#pragma once

#include <o80/standalone.hpp>
#include <real_time_tools/thread.hpp>

namespace o80
{
template <class RobotDriver, class o80Standalone>
THREAD_FUNCTION_RETURN_TYPE run_helper(void *arg);

template <class RobotDriver, class o80Standalone>
class StandaloneRunner
{
public:
    StandaloneRunner(std::string segment_id,
                     double max_action_duration_s,
                     double max_inter_action_duration_s,
                     double frequency,
                     bool bursting)
        : bursting_(bursting),
          running_(false),
          driver_(),
          standalone_(driver_,
                      max_action_duration_s,
                      max_inter_action_duration_s,
                      frequency,
                      segment_id)
    {
    }

    ~StandaloneRunner()
    {
        if (running_)
        {
            stop();
        }
    }

    void start()
    {
        standalone_.start();
        thread_.create_realtime_thread(run_helper, (void *)this);
    }

    void stop()
    {
        running_ = false;
        thread_.join();
        standalone_.stop();
    }

    void run()
    {
        bool should_run = true;
        while (running_ && should_run)
        {
            standalone.spin(bursting_);
        }
    }

private:
    bool bursting_;
    std::atomic<bool> running_;
    real_time_tools::Thread thread_;
    RobotDriver driver_;
    o80Standalone standalone_;
};

template <class RobotDriver, class o80Standalone>
THREAD_FUNCTION_RETURN_TYPE run_helper(void *arg)
{
    ((StandaloneRunner<RobotDriver, o80Standalone> *)arg)->run();
    return THREAD_FUNCTION_RETURN_VALUE;
}
}  // namespace o80
