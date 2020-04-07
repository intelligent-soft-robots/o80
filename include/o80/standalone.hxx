// Copyright (c) 2019 Max Planck Gesellschaft
// Author : Vincent Berenz

#define TEMPLATE_STANDALONE         \
    template <int QUEUE_SIZE,       \
              int NB_ACTUATORS,     \
              class RI_ACTION,      \
              class RI_OBSERVATION, \
              class o80_STATE,      \
              class o80_EXTENDED_STATE>

#define STANDALONE             \
    Standalone<QUEUE_SIZE,     \
               NB_ACTUATORS,   \
               RI_ACTION,      \
               RI_OBSERVATION, \
               o80_STATE,      \
               o80_EXTENDED_STATE>

static int get_bursting(const std::string& segment_id)
{
    int r;
    shared_memory::get<int>(segment_id, "bursting", r);
    return r;
}

static void reset_bursting(const std::string& segment_id)
{
    shared_memory::set<int>(segment_id, "bursting", 0);
}

TEMPLATE_STANDALONE
STANDALONE::Standalone(RiDriverPtr ri_driver_ptr,
                       double frequency,
                       std::string segment_id)
    : frequency_(frequency),
      period_(static_cast<long int>((1.0 / frequency + 0.5) * 10E6)),
      now_(time_now()),
      burster_(nullptr),
      segment_id_(segment_id),
      ri_driver_ptr_(ri_driver_ptr),
      ri_data_ptr_(std::make_shared<RiData>()),
      ri_frontend_(ri_data_ptr_),
      o8o_backend_(segment_id),
      ri_backend_ptr_(nullptr)
{
    shared_memory::set<bool>(segment_id, "should_stop", false);
}

TEMPLATE_STANDALONE
STANDALONE::~Standalone()
{
    stop();
}

TEMPLATE_STANDALONE
void STANDALONE::start()
{
    if (ri_backend_ptr_ == nullptr)
    {
        ri_backend_ptr_ = new RiBackend(ri_driver_ptr_,
                                        ri_data_ptr_,
					std::numeric_limits<double>::infinity(),
					std::numeric_limits<double>::infinity());
        ri_backend_ptr_->initialize();

        RI_ACTION action;
        ri_frontend_.append_desired_action(action);

        std::cout
            << "WARNING: o80 standalone start: setting random first action !\n";
    }
    else
    {
        throw std::runtime_error("a standalone should not be started twice");
    }
    spinner_.set_frequency(frequency_);
}

TEMPLATE_STANDALONE
void STANDALONE::stop()
{
    if (ri_backend_ptr_ != nullptr)
    {
        delete ri_backend_ptr_;
        ri_backend_ptr_ = nullptr;
    }
}

TEMPLATE_STANDALONE
bool STANDALONE::iterate(const TimePoint& time_now,
                         o80_EXTENDED_STATE& extended_state)
{
    // reading sensory info from the robot (robot_interfaces)

    robot_interfaces::TimeIndex time_index =
        ri_frontend_.get_current_timeindex();

    RI_OBSERVATION ri_current_states = ri_frontend_.get_observation(time_index);

    // converting robot_interfaces sensory reading to o80 state
    o80::States<NB_ACTUATORS, o80_STATE> o8o_current_states =
        convert(ri_current_states);

    // adding information to extended state, based on all what is available
    enrich_extended_state(extended_state, ri_current_states);

    // o80 machinery : reading the stack of command and using controller to
    // compute
    //                  desired state for each actuator, writing observation to
    //                  shared memory
    const o80::States<NB_ACTUATORS, o80_STATE>& desired_states =
        o8o_backend_.pulse(time_now, o8o_current_states, extended_state);

    // converting o80 desired state to action to input to robot interface
    RI_ACTION action = convert(desired_states);

    // applying actions to robot
    robot_interfaces::TimeIndex ti = ri_frontend_.append_desired_action(action);

    // TO DO : this is supposed to wait for the action to be applied (???)
    // does not work (there seems to be one iteration shift)
    ri_frontend_.wait_until_timeindex(ti);

    // check if stop command written by user in shared memory
    bool should_stop;
    shared_memory::get<bool>(segment_id_, "should_stop", should_stop);

    return !should_stop;
}

TEMPLATE_STANDALONE
bool STANDALONE::spin(o80_EXTENDED_STATE& extended_state, bool bursting)
{
    if (bursting && burster_ == nullptr)
    {
        burster_ = std::make_shared<Burster>(segment_id_);
    }

    int nb_iterations = 1;
    if (bursting)
    {
        nb_iterations = get_bursting(segment_id_);
        reset_bursting(segment_id_);
    }

    bool should_not_stop = true;

    for (int it = 0; it < nb_iterations; it++)
    {
        // one iteration (reading command, applying them, writing
        // observations to shared memory)

        should_not_stop = iterate(now_, extended_state);

        // received stop signal from user via shared memory
        if (!should_not_stop)
        {
            break;
        }

        // not in bursting, running at desired frequency
        if (!bursting)
        {
            spinner_.spin();
            now_ = time_now();
        }

        // bursting : running as fast as possible,
        // but keeping track of virtual time
        else
        {
            now_ += period_;
        }
    }

    // bursting : after burst of several iterations,
    // wait for client/python to ask to go again
    if (bursting && should_not_stop)
    {
        burster_->pulse();
        // follower_->pulse();
    }

    return should_not_stop;
}

TEMPLATE_STANDALONE
bool STANDALONE::spin(bool bursting)
{
    o80_EXTENDED_STATE empty;
    return spin(empty, bursting);
}

template <class RobotDriver, class o80Standalone, typename... Args>
void start_action_timed_standalone(std::string segment_id,
                                   double frequency,
                                   bool bursting,
                                   Args&&... args)
{
    if (internal::standalone_exists(segment_id))
    {
        std::string error = std::string("standalone ");
        error += segment_id;
        error += std::string(" already exists");
        throw std::runtime_error(error);
    }

    o80::clear_shared_memory(segment_id);

    typedef internal::StandaloneRunner<RobotDriver, o80Standalone> SR;
    typedef std::shared_ptr<SR> SRPtr;

    SRPtr runner(new SR(segment_id,
                        frequency,
                        bursting,
                        std::forward<Args>(args)...));
    runner->start();
    internal::add_standalone(segment_id, runner);
}

template <class RobotDriver, class o80Standalone,typename... Args>
void start_standalone(std::string segment_id,
                      double frequency,
                      bool bursting,
                      Args&&... args)
{
    start_action_timed_standalone<RobotDriver,
				  o80Standalone,
				  Args...>(
        segment_id,
        frequency,
        bursting,
        std::forward<Args>(args)...);
}

bool standalone_is_running(std::string segment_id)
{
    if (!internal::standalone_exists(segment_id))
    {
        return false;
    }
    internal::StandalonePtr& runner = internal::get_standalone(segment_id);
    return runner->is_running();
}

void stop_standalone(std::string segment_id)
{
    if (!internal::standalone_exists(segment_id))
    {
        std::string error = std::string("standalone ");
        error += segment_id;
        error += std::string(" does not exist");
        throw std::runtime_error(error);
    }

    internal::StandalonePtr& runner = internal::get_standalone(segment_id);

    runner->stop();

    o80::clear_shared_memory(segment_id);
}
