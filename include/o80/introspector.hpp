#pragma once

#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <real_time_tools/thread.hpp>
#include "o80/front_end.hpp"

namespace o80
{
namespace internal
{
template <class ROBOT_STATE>
THREAD_FUNCTION_RETURN_TYPE run_commands_helper(void *arg);
template <class ROBOT_STATE>
THREAD_FUNCTION_RETURN_TYPE run_completed_commands_helper(void *arg);
template <class ROBOT_STATE>
THREAD_FUNCTION_RETURN_TYPE run_waiting_for_completion_helper(void *arg);
template <class ROBOT_STATE>
THREAD_FUNCTION_RETURN_TYPE run_completion_reported_helper(void *arg);
template <class ROBOT_STATE>
THREAD_FUNCTION_RETURN_TYPE run_received_helper(void *arg);
template <class ROBOT_STATE>
THREAD_FUNCTION_RETURN_TYPE run_starting_helper(void *arg);
}  // namespace internal

template <class ROBOT_STATE>
class Introspector
{
public:
    typedef time_series::MultiprocessTimeSeries<Command<ROBOT_STATE>>
        CommandsTimeSeries;
    typedef time_series::MultiprocessTimeSeries<int>
        CompletedCommandsTimeSeries;

public:
    Introspector(std::string segment_id);
    void start();
    void stop();

private:
    friend THREAD_FUNCTION_RETURN_TYPE
    internal::run_commands_helper<ROBOT_STATE>(void *arg);
    friend THREAD_FUNCTION_RETURN_TYPE
    internal::run_completed_commands_helper<ROBOT_STATE>(void *arg);
    friend THREAD_FUNCTION_RETURN_TYPE
    internal::run_waiting_for_completion_helper<ROBOT_STATE>(void *arg);
    friend THREAD_FUNCTION_RETURN_TYPE
    internal::run_completion_reported_helper<ROBOT_STATE>(void *arg);
    friend THREAD_FUNCTION_RETURN_TYPE
    internal::run_received_helper<ROBOT_STATE>(void *arg);
    friend THREAD_FUNCTION_RETURN_TYPE
    internal::run_starting_helper<ROBOT_STATE>(void *arg);
    template <class T>
    void run(std::shared_ptr<T> time_series, std::string prefix);
    void run_commands();
    void run_completed_commands();
    void run_waiting_for_completion();
    void run_completion_reported();
    void run_received();
    void run_starting();

private:
    std::mutex mutex_;
    std::atomic<bool> running_;
    real_time_tools::RealTimeThread commands_thread_;
    real_time_tools::RealTimeThread completed_commands_thread_;
    real_time_tools::RealTimeThread waiting_for_completion_thread_;
    real_time_tools::RealTimeThread completion_reported_thread_;
    real_time_tools::RealTimeThread received_thread_;
    real_time_tools::RealTimeThread starting_thread_;
    std::shared_ptr<CommandsTimeSeries> commands_;
    std::shared_ptr<CompletedCommandsTimeSeries> completed_commands_;
    std::shared_ptr<CompletedCommandsTimeSeries> waiting_for_completion_;
    std::shared_ptr<CompletedCommandsTimeSeries> completion_reported_;
    std::shared_ptr<CompletedCommandsTimeSeries> received_;
    std::shared_ptr<CompletedCommandsTimeSeries> starting_;

private:
    static std::shared_ptr<Introspector<ROBOT_STATE>> instance_;

public:
    static bool start_running(std::string segment_id);
    static bool stop_running();
};

#include "introspector.hxx"

}  // namespace o80
