// Copyright (c) 2019 Max Planck Gesellschaft
// Author : Vincent Berenz

template <int QUEUE_SIZE,
          int NB_ACTUATORS,
          class o80_STATE,
          class o80_EXTENDED_STATE>
void create_core_python_bindings(pybind11::module &m,
				 Pybind11Config pybind11_config)
{

    if(pybind11_config.states)
	{
	    typedef States<NB_ACTUATORS, o80_STATE> states;
	    pybind11::class_<states>(m, (pybind11_config.prefix+"States").c_str())
		.def(pybind11::init<>())
		.def("set", &states::set)
		.def("get", &states::get)
		.def_readwrite("values", &states::values);
	}
    if (pybind11_config.state)
    {
        pybind11::class_<o80_STATE>(m, (pybind11_config.prefix+"State").c_str())
            .def(pybind11::init<>())
            .def("get", &o80_STATE::get)
            .def("set", &o80_STATE::set)
            .def("to_string", &o80_STATE::to_string);
    }

    if (pybind11_config.extended_state)
    {
        pybind11::class_<o80_EXTENDED_STATE>(m,
					     (pybind11_config.prefix+"ExtendedState").c_str())
            .def(pybind11::init<>());
    }

    if (pybind11_config.observation)
	{
	    typedef Observation<NB_ACTUATORS, o80_STATE, o80_EXTENDED_STATE>
		observation;
	    pybind11::class_<observation>(m,
					  (pybind11_config.prefix+"Observation").c_str())
		.def(pybind11::init<>())
		.def("get_observed_states", &observation::get_observed_states)
		.def("get_desired_states", &observation::get_desired_states)
		.def("get_extended_state", &observation::get_extended_state)
		.def("get_iteration", &observation::get_iteration)
		.def("get_frequency", &observation::get_frequency)
		.def("get_time_stamp", &observation::get_time_stamp)
		.def("__str__", &observation::to_string);
	}

    if (pybind11_config.frontend)
	{
	    typedef Observation<NB_ACTUATORS, o80_STATE, o80_EXTENDED_STATE>
		observation;
	    typedef FrontEnd<QUEUE_SIZE, NB_ACTUATORS, o80_STATE, o80_EXTENDED_STATE>
		frontend;
	    pybind11::class_<frontend>(m, (pybind11_config.prefix+"FrontEnd").c_str())
		.def(pybind11::init<std::string>())
		.def("start_logging",&frontend::start_logging)
		.def("get_nb_actuators",&frontend::get_nb_actuators)
		.def("get_current_iteration",&frontend::get_current_iteration)
		.def("get_history_since",&frontend::get_history_since)
		.def("get_latest",&frontend::get_latest)
		.def("wait_for_next",&frontend::wait_for_next)
		.def("reset_next_index",&frontend::reset_next_index)
		.def("add_command",
		     (void (frontend::*)(int, o80_STATE, Iteration, Mode)) &
		     frontend::add_command)
		.def("add_command",
		     (void (frontend::*)(int, o80_STATE, Mode)) & frontend::add_command)
		.def("add_command",
		     (void (frontend::*)(int, o80_STATE, Speed, Mode)) &
		     frontend::add_command)
		.def("burst", &frontend::burst)
		.def("final_burst", &frontend::final_burst)
		.def("pulse_and_wait", &frontend::pulse_and_wait)
		.def("read", &frontend::read)
		.def("pulse", (observation (frontend::*)(Iteration)) & frontend::pulse)
		.def("pulse", (observation (frontend::*)()) & frontend::pulse);
	}

    if (pybind11_config.backend)
	{
	    typedef BackEnd<QUEUE_SIZE, NB_ACTUATORS, o80_STATE, o80_EXTENDED_STATE>
		backend;
	    pybind11::class_<backend>(m, (pybind11_config.prefix+"BackEnd").c_str())
		.def(pybind11::init<std::string>())
		.def(pybind11::init<std::string,bool>())
		.def("start_logging",&backend::start_logging)
		.def("pulse",&backend::pulse)
		.def("pulse",[](backend& bc)
		     {
			 o80::TimePoint time_now = o80::time_now();
			 o80::States<NB_ACTUATORS,o80_STATE> states_;
			 o80_EXTENDED_STATE extended_state;
			 return bc.pulse(time_now,states_,extended_state);
		     })
		.def("pulse",[](backend& bc,
				o80_EXTENDED_STATE& extended_state)
		     {
			 o80::TimePoint time_now = o80::time_now();
			 o80::States<NB_ACTUATORS,o80_STATE> states_;
			 return bc.pulse(time_now,states_,extended_state);
		     })
		.def("pulse",[](backend& bc,
				const std::array<o80_STATE,NB_ACTUATORS>& dof_states)
		     {
			 o80::TimePoint time_now = o80::time_now();
			 o80::States<NB_ACTUATORS,o80_STATE> states;
			 o80_EXTENDED_STATE extended_state;
			 for (int dof=0;dof<NB_ACTUATORS;dof++)
			     {
				 states.set(dof,dof_states[dof]);
			     }
			 return bc.pulse(time_now,states,extended_state);
		     });

	}

}


template <int QUEUE_SIZE,
          int NB_ACTUATORS,
          class o80_STATE,
          class o80_EXTENDED_STATE>
void create_core_python_bindings(pybind11::module &m)
{
    Pybind11Config config;
    create_core_python_bindings<QUEUE_SIZE,
				NB_ACTUATORS,
				o80_STATE,
				o80_EXTENDED_STATE>(m,config);
}


template <int QUEUE_SIZE,
          int NB_ACTUATORS,
          class RI_ACTION,
          class RI_OBSERVATION,
          class o80_STATE,
          class RobotDriver,
          class RobotStandalone,
          class o80_EXTENDED_STATE,
          typename... DriverArgs>
void _create_python_bindings(pybind11::module &m,
			     Pybind11Config pybind11_config)
{
    // checking classes derives from the correct base classes

    typedef robot_interfaces::RobotDriver<RI_ACTION, RI_OBSERVATION> RI_DRIVER;
    static_assert(std::is_base_of<RI_DRIVER, RobotDriver>::value,
                  "o80 python bindings: RobotDriver must be derived from "
                  "robot_interfaces::RobotDriver");

    typedef Standalone<QUEUE_SIZE,
                       NB_ACTUATORS,
                       RI_ACTION,
                       RI_OBSERVATION,
                       o80_STATE,
                       o80_EXTENDED_STATE>
        standalone;
    static_assert(std::is_base_of<standalone, RobotStandalone>::value,
                  "o80 python bindings: RobotStandalone must "
                  "be derived from o80::Standalone");

    // creating python bindings for everything, except Standalone
    // and robot driver
    
    create_core_python_bindings<QUEUE_SIZE,
				NB_ACTUATORS,
				o80_STATE,
				o80_EXTENDED_STATE>(
						    m,pybind11_config);    
    // adding robot driver and standalone

    m.def("start_standalone",
          [](std::string segment_id,
             double frequency,
             bool bursting,
             DriverArgs... driver_args) {
              start_standalone<RobotDriver, RobotStandalone>(
                  segment_id, frequency, bursting,
		  (driver_args)...);
          });

    m.def("stop_standalone", &stop_standalone);

    m.def("standalone_is_running", &standalone_is_running);

    m.def("please_stop", &please_stop);
}

template <class RobotDriver, class RobotStandalone, typename... DriverArgs>
void create_python_bindings(pybind11::module &m,
			    Pybind11Config pybind11_config)
{
    _create_python_bindings<RobotStandalone::queue_size,
                            RobotStandalone::nb_actuators,
                            typename RobotStandalone::RiAction,
                            typename RobotStandalone::RiObservation,
                            typename RobotStandalone::o80State,
                            RobotDriver,
                            RobotStandalone,
                            typename RobotStandalone::o80ExtendedState,
                            DriverArgs...>(
					   m, pybind11_config);
}

template <class RobotDriver, class RobotStandalone, typename... DriverArgs>
void create_python_bindings(pybind11::module &m)
{
    Pybind11Config config;
    _create_python_bindings<RobotStandalone::queue_size,
                            RobotStandalone::nb_actuators,
                            typename RobotStandalone::RiAction,
                            typename RobotStandalone::RiObservation,
                            typename RobotStandalone::o80State,
                            RobotDriver,
                            RobotStandalone,
                            typename RobotStandalone::o80ExtendedState,
                            DriverArgs...>(
					   m, config);
}
