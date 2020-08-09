// Copyright (c) 2019 Max Planck Gesellschaft
// Author : Vincent Berenz

#include <o80/burster.hpp>
#include <o80/command_types.hpp>
#include <o80/front_end.hpp>
#include <o80/mode.hpp>
#include <o80/observation.hpp>
#include <o80/standalone.hpp>
#include <o80/states.hpp>

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/stl_bind.h>

#include <string>
#include <type_traits>

namespace o80
{
/* ! Configuration for the creating of python bindings.
 *   By default, bindings for these (templated) classes will
 *   binded: States, State, Observation, ExtendedState,
 *   FrontEnd and Backend. This class allows to inhibite the
 *   creation of bindings on selected classes, by setting the
 *   related bool value to false.
 *   This class also allows to add a prefix to the name of the
 *   binded class. For example, if the prefix is "MyRobot",
 *   the name of the FrontEnd binded class will be "MyRobotFrontEnd".
 */
class Pybind11Config
{
public:
    Pybind11Config(bool all_false = false);
    bool states = true;
    bool state = true;
    bool observation = true;
    bool extended_state = true;
    bool frontend = true;
    bool backend = true;
    std::string prefix;
};

/**
 * ! Creates in python module m python bindings
 * for the classes States, State, ExtendedState, Observation,
 * FrontEnd and Backend, except if inhibited by the
 * the instance of Pybind11Config. The name of the created
 * class will have the prefix specified by the config.
 */
template <int QUEUE_SIZE,
          int NB_ACTUATORS,
          class o80_STATE,
          class o80_EXTENDED_STATE>
void create_core_python_bindings(pybind11::module &m,
                                 Pybind11Config pybind11_config);

/**
 * ! Creates in python module m python bindings
 * for the classes States, State, ExtendedState, Observation,
 * FrontEnd and Backend.
 */
template <int QUEUE_SIZE,
          int NB_ACTUATORS,
          class o80_STATE,
          class o80_EXTENDED_STATE>
void create_core_python_bindings(pybind11::module &m);

template <int QUEUE_SIZE,
          int NB_ACTUATORS,
          class o80_STATE,
          class RobotDriver,
          class RobotStandalone,
          class o80_EXTENDED_STATE,
          typename... DriverArgs>
void _create_python_bindings(pybind11::module &m,
                             Pybind11Config pybind11_config);

/**
 * ! Creates the python bindings for the classes
 *   States, State, ExtendedState, Observation,
 *   FrontEnd and Backend (except of the one inhibited by
 *   the configuration), as well as to the class Standalone
 *   and of the functions start_standalone, stop_standalone,
 *   standalone_is_running and please_stop.
 *   @tparam DriverArgs : argument list required to instantiate
 *                        RobotDriver.
 */
template <class RobotDriver, class RobotStandalone, typename... DriverArgs>
void create_python_bindings(pybind11::module &m,
                            Pybind11Config pybind11_config);

#include "pybind11_helper.hxx"
}  // namespace o80
