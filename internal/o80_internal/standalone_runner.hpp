#pragma once

#include <atomic>
#include <o80/standalone.hpp>
#include <real_time_tools/thread.hpp>

namespace o80
{
namespace internal
{
template <class RobotDriver, class o80Standalone>
THREAD_FUNCTION_RETURN_TYPE run_helper(void* arg);

class StandaloneRunnerInterface
{
public:
    virtual void start() = 0;
    virtual void stop() = 0;
    virtual void run() = 0;
    virtual bool is_running() = 0;
};

template <class RobotDriver, class o80Standalone>
class StandaloneRunner : public StandaloneRunnerInterface
{
public:
    template <typename... Args>
    StandaloneRunner(std::string segment_id,
                     double frequency,
                     bool bursting,
                     Args&&... args);

    ~StandaloneRunner();

    void start();
    void stop();
    void run();

    bool is_running();

private:
    bool bursting_;
    std::atomic<bool> running_;
    real_time_tools::RealTimeThread thread_;
    std::shared_ptr<RobotDriver> driver_ptr_;
    o80Standalone standalone_;
};

typedef std::shared_ptr<StandaloneRunnerInterface> StandalonePtr;

StandalonePtr& get_standalone(const std::string& segment_id);
void add_standalone(const std::string& segment_id, StandalonePtr standalone);
bool standalone_exists(const std::string& segment_id);

#include "standalone_runner.hxx"
}
}
