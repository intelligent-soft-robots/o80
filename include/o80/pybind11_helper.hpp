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
typedef std::integral_constant<int, 0> NO_STATES;
typedef std::integral_constant<int, 1> NO_STATE;
typedef std::integral_constant<int, 2> NO_OBSERVATION;
typedef std::integral_constant<int, 3> NO_EXTENDED_STATE;
typedef std::integral_constant<int, 4> NO_FRONTEND;
typedef std::integral_constant<int, 5> NO_BACKEND;

/**
 * ! Creates in python module m python bindings
 * for the classes States, State, ExtendedState, Observation,
 * FrontEnd and Backend.
 */
template <int QUEUE_SIZE,
          int NB_ACTUATORS,
          class o80_STATE,
          class o80_EXTENDED_STATE,
          typename... EXCLUDED_CLASSES>
void create_python_bindings(pybind11::module &m,
                            std::string prefix = std::string(""));

template <class RobotStandalone, typename... EXCLUDED_CLASSES>
void create_python_bindings(pybind11::module &m,
                            std::string prefix = std::string(""));

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
void create_standalone_python_bindings(pybind11::module &m,
                                       std::string prefix = std::string(""));

#include "pybind11_helper.hxx"
}  // namespace o80
