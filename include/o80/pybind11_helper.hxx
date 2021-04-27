// Copyright (c) 2019 Max Planck Gesellschaft
// Author : Vincent Berenz

namespace internal
{
// returns true if T in Ts, false otherwise
template <typename T, typename... Ts>
constexpr bool has_type()
{
    return std::disjunction<std::is_same<T, Ts>...>::value;
}

}  // namespace internal

template <int QUEUE_SIZE,
          int NB_ACTUATORS,
          class o80_STATE,
          class o80_EXTENDED_STATE,
          typename... EXCLUDED_CLASSES>
void create_python_bindings(pybind11::module& m, std::string prefix)
{
    // binding States only if NO_STATES *not* passed as template
    // argument (similar comment for all the bindings below)
    if constexpr (!internal::has_type<NO_STATES, EXCLUDED_CLASSES...>())
    {
        typedef States<NB_ACTUATORS, o80_STATE> states;
        pybind11::class_<states>(m, (prefix + "States").c_str())
            .def(pybind11::init<>())
            .def("set", &states::set)
            .def("get", &states::get)
            .def_readwrite("values", &states::values);
    }
    if constexpr (!internal::has_type<NO_STATE, EXCLUDED_CLASSES...>())
    {
        pybind11::class_<o80_STATE>(m, (prefix + "State").c_str())
            .def(pybind11::init<>())
            .def("get", &o80_STATE::get)
            .def("set", &o80_STATE::set)
            .def("to_string", &o80_STATE::to_string);
    }

    if constexpr (!internal::has_type<NO_EXTENDED_STATE, EXCLUDED_CLASSES...>())
    {
        pybind11::class_<o80_EXTENDED_STATE>(m,
                                             (prefix + "ExtendedState").c_str())
            .def(pybind11::init<>());
    }

    if constexpr (!internal::has_type<NO_OBSERVATION, EXCLUDED_CLASSES...>())
    {
        typedef Observation<NB_ACTUATORS, o80_STATE, o80_EXTENDED_STATE>
            observation;
        pybind11::class_<observation>(m, (prefix + "Observation").c_str())
            .def(pybind11::init<>())
            .def("get_observed_states", &observation::get_observed_states)
            .def("get_desired_states", &observation::get_desired_states)
            .def("get_extended_state", &observation::get_extended_state)
            .def("get_iteration", &observation::get_iteration)
            .def("get_frequency", &observation::get_frequency)
            .def("get_time_stamp", &observation::get_time_stamp)
            .def("__str__", &observation::to_string);
    }

    if constexpr (!internal::has_type<NO_SERIALIZER, EXCLUDED_CLASSES...>())
    {
        typedef Observation<NB_ACTUATORS, o80_STATE, o80_EXTENDED_STATE>
            observation;
        typedef shared_memory::Serializer<observation> serializer;
        pybind11::class_<serializer>(m, (prefix + "Serializer").c_str())
            .def(pybind11::init<>())
            .def("serializable_size", &serializer::serializable_size)
            .def("serialize",
                 [](serializer& s, const observation& obs) {
                     return s.serialize(obs);
                 })
            .def("deserialize", &serializer::deserialize);
    }

    if constexpr (!internal::has_type<NO_FRONTEND, EXCLUDED_CLASSES...>())
    {
        typedef Observation<NB_ACTUATORS, o80_STATE, o80_EXTENDED_STATE>
            observation;
        typedef FrontEnd<QUEUE_SIZE,
                         NB_ACTUATORS,
                         o80_STATE,
                         o80_EXTENDED_STATE>
            frontend;
        pybind11::class_<frontend>(m, (prefix + "FrontEnd").c_str())
            .def(pybind11::init<std::string>())
            .def("get_nb_actuators", &frontend::get_nb_actuators)
            .def("get_observations_since", &frontend::get_observations_since)
            .def("get_latest_observations", &frontend::get_latest_observations)
            .def("wait_for_next", &frontend::wait_for_next)
            .def("reset_next_index", &frontend::reset_next_index)
            .def("is_backend_active", &frontend::backend_is_active)
            .def("purge", &frontend::purge)
            .def("add_command",
                 (void (frontend::*)(int, o80_STATE, Iteration, Mode)) &
                     frontend::add_command)
            .def("add_command",
                 (void (frontend::*)(int, o80_STATE, Duration_us, Mode)) &
                     frontend::add_command)
            .def("add_command",
                 (void (frontend::*)(int, o80_STATE, Mode)) &
                     frontend::add_command)
            .def("add_command",
                 (void (frontend::*)(int, o80_STATE, Speed, Mode)) &
                     frontend::add_command)
            .def("burst", &frontend::burst)
            .def("final_burst", &frontend::final_burst)
            .def("pulse_and_wait", &frontend::pulse_and_wait)
            .def("pulse_prepare_wait", &frontend::pulse_prepare_wait)
            .def("wait", &frontend::wait)
            .def("read", &frontend::read)
            .def("latest", [](frontend& fe) { return fe.read(-1); })
            .def("pulse",
                 (observation(frontend::*)(Iteration)) & frontend::pulse)
            .def("pulse", (observation(frontend::*)()) & frontend::pulse);
    }

    if constexpr (!internal::has_type<NO_BACKEND, EXCLUDED_CLASSES...>())
    {
        typedef BackEnd<QUEUE_SIZE, NB_ACTUATORS, o80_STATE, o80_EXTENDED_STATE>
            backend;
        pybind11::class_<backend>(m, (prefix + "BackEnd").c_str())
            .def(pybind11::init<std::string>())
            .def(pybind11::init<std::string, bool>())
            .def("is_active", &backend::is_active)
            .def("pulse", &backend::pulse)
            .def("pulse",
                 [](backend& bc) {
                     o80::TimePoint time_now = o80::time_now();
                     o80::States<NB_ACTUATORS, o80_STATE> states_;
                     o80_EXTENDED_STATE extended_state;
                     return bc.pulse(time_now, states_, extended_state);
                 })
            .def("pulse",
                 [](backend& bc, o80_EXTENDED_STATE& extended_state) {
                     o80::TimePoint time_now = o80::time_now();
                     o80::States<NB_ACTUATORS, o80_STATE> states_;
                     return bc.pulse(time_now, states_, extended_state);
                 })
            .def("pulse",
                 [](backend& bc,
                    const std::array<o80_STATE, NB_ACTUATORS>& dof_states) {
                     o80::TimePoint time_now = o80::time_now();
                     o80::States<NB_ACTUATORS, o80_STATE> states;
                     o80_EXTENDED_STATE extended_state;
                     for (int dof = 0; dof < NB_ACTUATORS; dof++)
                     {
                         states.set(dof, dof_states[dof]);
                     }
                     return bc.pulse(time_now, states, extended_state);
                 });
    }
    if constexpr (!internal::has_type<NO_INTROSPECTOR, EXCLUDED_CLASSES...>())
    {
        typedef Introspector<o80_STATE> introspector;
        pybind11::class_<introspector>(m, (prefix + "Introspector").c_str())
            .def_static((prefix + std::string("start")).c_str(),
                        &introspector::start_running)
            .def_static((prefix + std::string("stop")).c_str(),
                        &introspector::stop_running);
    }
}

template <class RobotStandalone, typename... EXCLUDED_CLASSES>
void create_python_bindings(pybind11::module& m, std::string prefix)
{
    create_python_bindings<RobotStandalone::queue_size,
                           RobotStandalone::nb_actuators,
                           typename RobotStandalone::o80State,
                           typename RobotStandalone::o80ExtendedState,
                           EXCLUDED_CLASSES...>(m, prefix);
}

template <class RobotDriver, class RobotStandalone, typename... DriverArgs>
void create_standalone_python_bindings(pybind11::module& m, std::string prefix)
{
    m.def((prefix + std::string("start_standalone")).c_str(),
          [](std::string segment_id,
             double frequency,
             bool bursting,
             DriverArgs... driver_args) {
              start_standalone<RobotDriver, RobotStandalone>(
                  segment_id, frequency, bursting, (driver_args)...);
          });

    m.def("stop_standalone", &stop_standalone);

    m.def("standalone_is_running", &standalone_is_running);

    m.def("please_stop", &please_stop);
}
