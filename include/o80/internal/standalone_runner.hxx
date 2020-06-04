#define SRUNNER StandaloneRunner<RobotDriver, o80Standalone>

template <class RobotDriver, class o80Standalone>
template <typename... Args>
SRUNNER::StandaloneRunner(std::string segment_id,
                          double frequency,
                          bool bursting,
                          Args&&... args)
    : bursting_(bursting),
      running_(false),
      driver_ptr_(std::make_shared<RobotDriver>(std::forward<Args>(args)...)),
      standalone_(driver_ptr_, frequency, segment_id)
{
}

template <class RobotDriver, class o80Standalone>
SRUNNER::~StandaloneRunner()
{
    if (running_)
    {
        stop();
    }
}

template <class RobotDriver, class o80Standalone>
void SRUNNER::start()
{
    driver_ptr_->initialize();
    standalone_.start();
    thread_.create_realtime_thread(run_helper<RobotDriver, o80Standalone>,
                                   (void*)this);
}

template <class RobotDriver, class o80Standalone>
void SRUNNER::stop()
{
    running_ = false;
    thread_.join();
}

template <class RobotDriver, class o80Standalone>
void SRUNNER::run()
{
    running_ = true;
    bool should_run = true;
    while (running_ && should_run)
    {
        should_run = standalone_.spin(bursting_);
    }
    running_ = false;
    standalone_.stop();
}

template <class RobotDriver, class o80Standalone>
bool SRUNNER::is_running()
{
    return running_;
}

template <class RobotDriver, class o80Standalone>
THREAD_FUNCTION_RETURN_TYPE run_helper(void* arg)
{
    ((StandaloneRunner<RobotDriver, o80Standalone>*)arg)->run();
    return THREAD_FUNCTION_RETURN_VALUE;
}

static std::map<std::string, StandalonePtr> standalones;

StandalonePtr& get_standalone(const std::string& segment_id)
{
    StandalonePtr& runner = standalones.at(segment_id);
    return runner;
}

void add_standalone(const std::string& segment_id, StandalonePtr standalone)
{
    standalones.insert(
        std::pair<std::string, StandalonePtr>(segment_id, standalone));
}

bool standalone_exists(const std::string& segment_id)
{
    if (standalones.find(segment_id) == standalones.end())
    {
        return false;
    }
    return true;
}
