#include "o80/back_end.hpp"
#include "o80/bool_state.hpp"
#include "o80/burster.hpp"
#include "o80/command_types.hpp"
#include "o80/frequency_manager.hpp"
#include "o80/frequency_measure.hpp"
#include "o80/item3d_state.hpp"
#include "o80/memory_clearing.hpp"
#include "o80/pybind11_helper.hpp"
#include "o80/state1d.hpp"
#include "o80/state2d.hpp"
#include "o80/state3d.hpp"
#include "o80/state6d.hpp"
#include "o80/time.hpp"

// are wrapped here only the non templated class if o80.
// For bindings of templated classes, see o80/pybind_helper.hpp

using namespace o80;

PYBIND11_MODULE(o80, m)
{
    m.def("clear_shared_memory", &clear_shared_memory);

    m.def("time_now", &time_now);

    m.def("time_diff", [](const TimePoint& before, const TimePoint& after) {
        return time_diff(before, after);
    });

    pybind11::class_<o80::Burster>(m, "Burster")
        .def(pybind11::init<std::string>())
        .def("pulse", &Burster::pulse)
        .def("clear_memory", &Burster::clear_memory)
        .def("turn_on", &Burster::turn_on)
        .def("turn_off", &Burster::turn_off);

    pybind11::class_<o80::BursterClient>(m, "BursterClient")
        .def(pybind11::init<std::string>())
        .def("burst", &BursterClient::burst)
        .def("final_burst", &BursterClient::final_burst);

    pybind11::class_<o80::BoolState>(m, "BoolState")
        .def(pybind11::init<bool>())
        .def("set", &BoolState::set)
        .def("get", &BoolState::get)
        .def("to_string", &BoolState::to_string);

    pybind11::class_<o80::Iteration>(m, "Iteration")
        .def(pybind11::init<long int>())
        .def(pybind11::init<long int, bool>())
        .def(pybind11::init<long int, bool, bool>())
        .def("reset", &Iteration::reset);

    pybind11::class_<o80::Direct>(m, "Direct").def(pybind11::init<>());

    pybind11::class_<o80::Duration_us>(m, "Duration_us")
        .def("seconds", o80::Duration_us::seconds)
        .def("milliseconds", o80::Duration_us::milliseconds)
        .def("microseconds", o80::Duration_us::microseconds)
        .def("nanoseconds", o80::Duration_us::nanoseconds);

    pybind11::class_<o80::Speed>(m, "Speed")
        .def("per_second", o80::Speed::per_second)
        .def("per_millisecond", o80::Speed::per_millisecond)
        .def("per_microsecond", o80::Speed::per_microsecond)
        .def("per_nanosecond", o80::Speed::per_nanosecond);

    pybind11::enum_<o80::Mode>(m, "Mode")
        .value("QUEUE", o80::QUEUE)
        .value("OVERWRITE", o80::OVERWRITE);

    pybind11::enum_<o80::Type>(m, "Type")
        .value("DURATION", o80::DURATION)
        .value("SPEED", o80::SPEED)
        .value("DIRECT", o80::DIRECT)
        .value("ITERATION", o80::ITERATION);

    pybind11::enum_<o80::LogAction>(m, "LogAction")
        .value("FRONTEND_WAIT_START", o80::FRONTEND_WAIT_START)
        .value("FRONTEND_WAIT_END", o80::FRONTEND_WAIT_END)
        .value("FRONTEND_COMMUNICATE", o80::FRONTEND_COMMUNICATE)
        .value("FRONTEND_READ", o80::FRONTEND_READ)
        .value("FRONTEND_COMPLETION_WAIT_START",
               o80::FRONTEND_COMPLETION_WAIT_START)
        .value("FRONTEND_COMPLETION_WAIT_END",
               o80::FRONTEND_COMPLETION_WAIT_END)
        .value("BACKEND_READ", o80::BACKEND_READ)
        .value("BACKEND_WRITE_REAPPLY", o80::BACKEND_WRITE_REAPPLY)
        .value("BACKEND_WRITE_NEW", o80::BACKEND_WRITE_NEW);

    pybind11::class_<o80::FrequencyMeasure>(m, "FrequencyMeasure")
        .def(pybind11::init<>())
        .def("tick", &FrequencyMeasure::tick);

    pybind11::class_<o80::FrequencyManager>(m, "FrequencyManager")
        .def(pybind11::init<double>())
        .def("wait", &FrequencyManager::wait);

    pybind11::class_<o80::State1d>(m, "State1d")
        .def(pybind11::init<>())
        .def(pybind11::init<double>())
        .def("__str__", &State1d::to_string)
        .def("get", &State1d::get)
        .def("set", &State1d::set);

    pybind11::class_<o80::State2d>(m, "State2d")
        .def(pybind11::init<>())
        .def(pybind11::init<double, double>())
        .def("__str__", &State2d::to_string)
        .def("get",
             [](o80::State2d& state2d, int index) {
                 if (index == 0) return state2d.get<0>();
                 return state2d.get<1>();
             })
        .def("set", [](o80::State2d& state2d, int index, double value) {
            if (index == 0)
            {
                state2d.set<0>(value);
                return;
            }
            state2d.set<1>(value);
        });

    pybind11::class_<o80::State3d>(m, "State3d")
        .def(pybind11::init<>())
        .def(pybind11::init<double, double, double>())
        .def("__str__", &State3d::to_string)
        .def("get",
             [](o80::State3d& state3d, int index) {
                 if (index == 0) return state3d.get<0>();
                 if (index == 1) return state3d.get<1>();
                 return state3d.get<2>();
             })
        .def("set", [](o80::State3d& state3d, int index, double value) {
            if (index == 0)
            {
                state3d.set<0>(value);
                return;
            }
            if (index == 1)
            {
                state3d.set<1>(value);
                return;
            }
            state3d.set<2>(value);
        });

    pybind11::class_<o80::Item3dState>(m, "Item3dState")
        .def(pybind11::init<>())
        .def(pybind11::init<const std::array<double, 3>&,
                            const std::array<double, 3>&>())
        .def("set_position",
             pybind11::overload_cast<double, double, double>(
                 &Item3dState::set_position))
        .def("set_velocity",
             pybind11::overload_cast<double, double, double>(
                 &Item3dState::set_velocity))
        .def("set_position",
             pybind11::overload_cast<const std::array<double, 3>&>(
                 &Item3dState::set_position))
        .def("set_velocity",
             pybind11::overload_cast<const std::array<double, 3>&>(
                 &Item3dState::set_velocity))
        .def("get_position", &Item3dState::get_position)
        .def("get_velocity", &Item3dState::get_velocity)
        .def("get", &Item3dState::get)
        .def("__str__", &Item3dState::to_string);

    pybind11::class_<o80::State6d>(m, "State6d")
        .def(pybind11::init<>())
        .def(pybind11::init<double, double, double, double, double, double>())
        .def("__str__",
             [](const o80::State6d& state6d) {
                 std::stringstream ss;
                 ss << "position: " << state6d.get<0>() << ","
                    << state6d.get<1>() << "," << state6d.get<2>() << " | "
                    << "velocity: " << state6d.get<3>() << ","
                    << state6d.get<4>() << "," << state6d.get<5>();
                 return ss.str();
             })
        .def("get_position",
             [](o80::State6d& state6d) {
                 std::array<double, 3> position;
                 position[0] = state6d.get<0>();
                 position[1] = state6d.get<1>();
                 position[2] = state6d.get<2>();
                 return position;
             })
        .def("get_velocity",
             [](o80::State6d& state6d) {
                 std::array<double, 3> velocity;
                 velocity[0] = state6d.get<3>();
                 velocity[1] = state6d.get<4>();
                 velocity[2] = state6d.get<5>();
                 return velocity;
             })
        .def("set_position",
             [](o80::State6d& state6d, const std::array<double, 3>& position) {
                 state6d.set<0>(position[0]);
                 state6d.set<1>(position[1]);
                 state6d.set<2>(position[2]);
                 return;
             })
        .def("set_velocity",
             [](o80::State6d& state6d, const std::array<double, 3>& velocity) {
                 state6d.set<3>(velocity[0]);
                 state6d.set<4>(velocity[1]);
                 state6d.set<5>(velocity[2]);
                 return;
             })
        .def("get",
             [](o80::State6d& state6d, int index) {
                 if (index == 0) return state6d.get<0>();
                 if (index == 1) return state6d.get<1>();
                 if (index == 2) return state6d.get<2>();
                 if (index == 3) return state6d.get<3>();
                 if (index == 4) return state6d.get<4>();
                 return state6d.get<5>();
             })
        .def("set", [](o80::State6d& state6d, int index, double value) {
            if (index == 0)
            {
                state6d.set<0>(value);
                return;
            }
            if (index == 1)
            {
                state6d.set<1>(value);
                return;
            }
            if (index == 2)
            {
                state6d.set<2>(value);
                return;
            }
            if (index == 3)
            {
                state6d.set<3>(value);
                return;
            }
            if (index == 4)
            {
                state6d.set<4>(value);
                return;
            }
            state6d.set<5>(value);
        });
}
