// Copyright (c) 2019 Max Planck Gesellschaft
// Author : Vincent Berenz

#define TEMPLATE_STANDALONE     \
    template <int QUEUE_SIZE,   \
              int NB_ACTUATORS, \
              class DRIVER,     \
              class o80_STATE,  \
              class o80_EXTENDED_STATE>

#define STANDALONE \
    Standalone<QUEUE_SIZE, NB_ACTUATORS, DRIVER, o80_STATE, o80_EXTENDED_STATE>

static long int get_bursting(const std::string& segment_id)
{
    long int r;
    shared_memory::get<long int>(segment_id, "bursting", r);
    return r;
}

static void reset_bursting(const std::string& segment_id)
{
    shared_memory::set<long int>(segment_id, "bursting", 0);
}

TEMPLATE_STANDALONE
STANDALONE::Standalone(DriverPtr driver_ptr,
                       double frequency,
                       std::string segment_id)
    : frequency_(frequency),
      period_(static_cast<long int>((1.0 / frequency) * 1E6 + 0.5)),
      frequency_manager_(frequency_),
      now_(time_now()),
      burster_(nullptr),
      segment_id_(segment_id),
      driver_ptr_(driver_ptr),
      o8o_backend_(segment_id)
{
    shared_memory::set<bool>(segment_id, "should_stop", false);
    reset_bursting(segment_id);
}

TEMPLATE_STANDALONE
STANDALONE::~Standalone()
{
    stop();
}

TEMPLATE_STANDALONE
void STANDALONE::start()
{
    driver_ptr_->start();
}

TEMPLATE_STANDALONE
void STANDALONE::stop()
{
    driver_ptr_->stop();
}

TEMPLATE_STANDALONE
bool STANDALONE::iterate(const TimePoint& time_now,
                         o80_EXTENDED_STATE& extended_state)
{
    // reading sensory info from the robot (robot_interfaces)
    typename DRIVER::DRIVER_OUT ri_current_states = driver_ptr_->get();

    // converting robot_interfaces sensory reading to o80 state
    o80::States<NB_ACTUATORS, o80_STATE> o8o_current_states =
        convert(ri_current_states);

    // adding information to extended state, based on all what is available
    enrich_extended_state(extended_state, ri_current_states);

    // o80 machinery : reading the queue of command and using controller to
    // compute
    //                  desired state for each actuator, writing observation to
    //                  shared memory
    const o80::States<NB_ACTUATORS, o80_STATE>& desired_states =
        o8o_backend_.pulse(time_now, o8o_current_states, extended_state);

    // converting o80 desired state to action to input to robot interface
    typename DRIVER::DRIVER_IN action = convert(desired_states);

    // applying actions to robot
    driver_ptr_->set(action);

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

    long int nb_iterations = 1;
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
            frequency_manager_.wait();
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
    }

    return should_not_stop;
}

TEMPLATE_STANDALONE
bool STANDALONE::spin(bool bursting)
{
    o80_EXTENDED_STATE empty;
    return spin(empty, bursting);
}

template <class Driver, class o80Standalone, typename... Args>
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

    typedef internal::StandaloneRunner<Driver, o80Standalone> SR;
    typedef std::shared_ptr<SR> SRPtr;

    SRPtr runner(
        new SR(segment_id, frequency, bursting, std::forward<Args>(args)...));
    runner->start();
    internal::add_standalone(segment_id, runner);
}

template <class Driver, class o80Standalone, typename... Args>
void start_standalone(std::string segment_id,
                      double frequency,
                      bool bursting,
                      Args&&... args)
{
    start_action_timed_standalone<Driver, o80Standalone, Args...>(
        segment_id, frequency, bursting, std::forward<Args>(args)...);
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
