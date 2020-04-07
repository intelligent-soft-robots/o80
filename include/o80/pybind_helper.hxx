// Copyright (c) 2019 Max Planck Gesellschaft
// Author : Vincent Berenz

template <int QUEUE_SIZE,
          int NB_ACTUATORS,
          class o80_STATE,
          class o80_EXTENDED_STATE>
void create_min_python_bindings(pybind11::module &m,
				bool states,
                                bool state,
                                bool extended_state)
{

    if(states)
	{
	    typedef States<NB_ACTUATORS, o80_STATE> states;
	    pybind11::class_<states>(m, "States")
		.def(pybind11::init<>())
		.def("set", &states::set)
		.def("get", &states::get)
		.def_readwrite("values", &states::values);
	}
    if (state)
    {
        pybind11::class_<o80_STATE>(m, "State")
            .def(pybind11::init<>())
            .def("get", &o80_STATE::get)
            .def("set", &o80_STATE::set)
            .def("to_string", &o80_STATE::to_string);
    }

    if (extended_state)
    {
        pybind11::class_<o80_EXTENDED_STATE>(m, "ExtendedState")
            .def(pybind11::init<>());
    }

    typedef Observation<NB_ACTUATORS, o80_STATE, o80_EXTENDED_STATE>
        observation;
    pybind11::class_<observation>(m, "Observation")
        .def(pybind11::init<>())
        .def("get_observed_states", &observation::get_observed_states)
        .def("get_desired_states", &observation::get_desired_states)
        .def("get_extended_state", &observation::get_extended_state)
        .def("get_iteration", &observation::get_iteration)
        .def("get_frequency", &observation::get_frequency)
        .def("__str__", &observation::to_string);

    typedef FrontEnd<QUEUE_SIZE, NB_ACTUATORS, o80_STATE, o80_EXTENDED_STATE>
        frontend;
    pybind11::class_<frontend>(m, "FrontEnd")
        .def(pybind11::init<std::string>())
	.def("get_current_iteration",&frontend::get_current_iteration)
	.def("get_history_since",&frontend::get_history_since)
	.def("get_latest",&frontend::get_latest)
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
        .def("pulse", (observation (frontend::*)(Iteration)) & frontend::pulse)
        .def("pulse", (observation (frontend::*)()) & frontend::pulse)
        .def("pulse_and_wait", &frontend::pulse_and_wait)
        .def("read", &frontend::read);
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
			     bool states,
                             bool state,
                             bool extended_state,
                             bool min_bindings)
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

    if (min_bindings)
    {
        // creating python bindings for everything, except Standalone
        // and robot driver

        create_min_python_bindings<QUEUE_SIZE,
                                   NB_ACTUATORS,
                                   o80_STATE,
                                   o80_EXTENDED_STATE>(
						       m, states, state, extended_state);
    }

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
			    bool states,
                            bool state,
                            bool extended_state,
                            bool min_bindings)
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
					   m, states, state, extended_state, min_bindings);
}
