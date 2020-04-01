// Copyright (c) 2019 Max Planck Gesellschaft
// Author : Vincent Berenz

#include <o80/burster.hpp>
#include <o80/front_end.hpp>
#include <o80/mode.hpp>
#include <o80/observation.hpp>
#include <o80/standalone.hpp>
#include <o80/states.hpp>
#include <o80/type.hpp>

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/stl_bind.h>

#include <string>
#include <type_traits>

namespace o80
{
/**
 * @brief creates in python module m python bindings
 * for Burster, Iteration, Direct, Duration_us, Speed,
 * Mode, Type, States, ExtendedState, Observation and
 * FrontEnd.
 */
template <int QUEUE_SIZE,
          int NB_ACTUATORS,
          class o80_STATE,
          class o80_EXTENDED_STATE>
void create_min_python_bindings(pybind11::module &m,
                                bool state = true,
                                bool extended_state = true);

/**
 * @brief creates in python module m python bindings
 * for Burster, Iteration, Direct, Duration_us, Speed,
 * Mode, Type, States, ExtendedState, Observation and
 * FrontEnd, start_standalone, stop_standalone
 */
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
                             bool state = true,
                             bool extended_state = true,
                             bool min_bindings = true);

template <class RobotDriver, class RobotStandalone, typename... DriverArgs>
void create_python_bindings(pybind11::module &m,
                            bool state = true,
                            bool extended_state = true,
                            bool min_bindings = true);

#include "pybind_helper.hxx"
}
