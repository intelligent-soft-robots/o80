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

    class Pybind11Config
    {
    public:
	Pybind11Config()
	    : prefix("")
	{}
	bool states = true;
	bool state = true;
	bool observation = true;
	bool extended_state = true;
	bool frontend = true;
	bool backend = true;
	std::string prefix;
    };
    
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
void create_core_python_bindings(pybind11::module &m,
				 Pybind11Config pybind11_config);

template <int QUEUE_SIZE,
          int NB_ACTUATORS,
          class o80_STATE,
          class o80_EXTENDED_STATE>
void create_core_python_bindings(pybind11::module &m);
    
    
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
			     Pybind11Config pybind11_config);

template <class RobotDriver, class RobotStandalone, typename... DriverArgs>
void create_python_bindings(pybind11::module &m,
			    Pybind11Config pybind11_config);

#include "pybind11_helper.hxx"

}
