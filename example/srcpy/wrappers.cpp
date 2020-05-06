#include "o80/example/example.hpp"
#include "o80/pybind11_helper.hpp"

PYBIND11_MODULE(o80_example, m)
{
    o80::create_python_bindings<o80_example::Driver,
                                o80_example::Standalone,
                                int,
                                int>  // int,int are arguments to drivers
        (m);
}
