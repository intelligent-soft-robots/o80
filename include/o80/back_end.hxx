// Copyright (c) 2019 Max Planck Gesellschaft
// Author : Vincent Berenz

#define TEMPLATE_BACKEND        \
    template <int QUEUE_SIZE,   \
              int NB_ACTUATORS, \
              class STATE,      \
              class EXTENDED_STATE>

#define BACKEND BackEnd<QUEUE_SIZE, NB_ACTUATORS, STATE, EXTENDED_STATE>

TEMPLATE_BACKEND
BACKEND::BackEnd(std::string segment_id, bool new_commands_observations)
    : segment_id_(segment_id),
      observations_{ObservationsTimeSeries::create_leader(
          segment_id + "_observations", QUEUE_SIZE)},
      controllers_manager_(segment_id),
      desired_states_(),
      iteration_(0),
      observed_frequency_(-1),
      new_commands_observations_(new_commands_observations),
      reapplied_desired_states_{true},
      waiting_for_completion_{CompletedCommandsTimeSeries::create_leader(
									 segment_id + "_waiting_for_completion",QUEUE_SIZE)},
      completion_reported_{CompletedCommandsTimeSeries::create_leader(
								     segment_id + "_completion_reported",QUEUE_SIZE)}

{
    frequency_measure_.tick();
    // this will be set to true when iterations do not reapply desired
    // states (i.e. at least one command is active), to false when
    // desired states is reapplied (no command is active)
    shared_memory::set<bool>(segment_id, "active", false);
    // frontend(s) may set this value to "true" to trigger
    // the purge of all commands
    shared_memory::set<bool>(segment_id,"purge",false);
}

TEMPLATE_BACKEND
BACKEND::~BackEnd()
{
    clear_shared_memory(segment_id_);
}

TEMPLATE_BACKEND
bool BACKEND::iterate(const TimePoint& time_now,
                      const States<NB_ACTUATORS, STATE>& current_states,
                      bool iteration_update,
                      long int current_iteration)
{
    if (!iteration_update)
    {
        iteration_ = current_iteration;
    }

    // checking if a frontend requested the purge of commands
    bool must_purge;
    shared_memory::get<bool>(segment_id_,"purge",must_purge);
    if(must_purge)
      {
	controllers_manager_.purge();
	shared_memory::set<bool>(segment_id_,"purge",true);
      }
    
    controllers_manager_.process_commands(iteration_);

    // reading desired state based on controllers output
    for (int controller_nb = 0; controller_nb < desired_states_.values.size();
         controller_nb++)
    {
        desired_states_.values[controller_nb] =
            controllers_manager_.get_desired_state(
                controller_nb,
                iteration_,
                time_now,
                current_states.values[controller_nb]);
    }

    observed_frequency_ = frequency_measure_.tick();

    return controllers_manager_.reapplied_desired_states();
}

TEMPLATE_BACKEND
bool BACKEND::is_active()
{
    return (!reapplied_desired_states_);
}

TEMPLATE_BACKEND
const States<NB_ACTUATORS, STATE>& BACKEND::pulse(
    const TimePoint& time_now,
    const States<NB_ACTUATORS, STATE>& current_states,
    const EXTENDED_STATE& extended_state,
    bool iteration_update,
    long int current_iteration)
{
    reapplied_desired_states_ =
        iterate(time_now, current_states, iteration_update, current_iteration);

    // for the sake of frontend::backend_is_active
    if (reapplied_desired_states_)
    {
        shared_memory::set<bool>(segment_id_, "active", false);
    }
    else
    {
        shared_memory::set<bool>(segment_id_, "active", true);
    }

    bool print_obs = true;
    if (new_commands_observations_)
    {
        if (reapplied_desired_states_)
        {
            print_obs = false;
        }
    }

    // writting current states to shared memory
    if (print_obs)
    {
        Observation<NB_ACTUATORS, STATE, EXTENDED_STATE> observation(
            current_states,
            desired_states_,
            extended_state,
            time_now.count(),
            iteration_,
            observed_frequency_);
        observations_.append(observation);
    }

    if (iteration_update)
    {
        iteration_++;
    }

    return desired_states_;
}
