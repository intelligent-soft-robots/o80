#include "o80/bool_state.hpp"
#include "o80/burster.hpp"
#include "o80/pybind_helper.hpp"
#include "o80/type.hpp"
#include "o80/typedefs.hpp"

// are wrapped here only the non templated class if o80.
// For bindings of templated classes, see o80/pybind_helper.hpp

using namespace o80;

PYBIND11_MODULE(o80_py, m)
{
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

    pybind11::class_<o80::BoolState>(m, "BoolState")
        .def(pybind11::init<bool>())
        .def("set", &BoolState::set)
        .def("get", &BoolState::get)
        .def("to_string", &BoolState::to_string);

    pybind11::class_<o80::Iteration>(m, "Iteration")
        .def(pybind11::init<long int>());

    pybind11::class_<o80::Direct>(m, "Direct").def(pybind11::init<>());

    pybind11::class_<o80::Duration_us>(m, "Duration_us")
        .def(pybind11::init<long int>());

    pybind11::class_<o80::Speed>(m, "Speed").def(pybind11::init<double>());

    pybind11::enum_<o80::Mode>(m, "Mode")
        .value("STACK", o80::QUEUE)
        .value("OVERWRITE", o80::OVERWRITE);

    pybind11::enum_<o80::Type>(m, "Type")
        .value("DURATION", o80::DURATION)
        .value("SPEED", o80::SPEED)
        .value("DIRECT", o80::DIRECT)
        .value("DELAYED", o80::DELAYED)
        .value("ITERATION", o80::ITERATION);
}
