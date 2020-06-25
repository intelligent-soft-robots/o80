
# Integrating your hardware

 [o80 example](https://github.com/intelligent-soft-robots/o80_example) provides an example of (pseudo toy) hardware integration. Here are the main steps to follow.

## Step1: Robot driver

To integrate your hardware, you need to code its corresponding Driver, which should be sublcass of [o80::Driver](https://github.com/intelligent-soft-robots/o80/blob/master/include/o80/driver.hpp).

A driver simply provides functions to communicate with the hardware, i.e. to send "action" (arbitrary user templated class IN)  and read sensors (arbitrary user templated class OUT).

Here is a toy examples of a driver:

- [driver.hpp](https://github.com/intelligent-soft-robots/o80_example/blob/master/include/o80_example/driver.hpp)
- [driver.cpp](https://github.com/intelligent-soft-robots/o80_example/blob/master/src/driver.cpp)


## Step2: o80 Standalone

An o80 [Standalone](https://github.com/intelligent-soft-robots/o80/blob/vberenz/doc/include/o80/driver.hpp) is an object that will wrap an instance of Driver so to make it compatible with the o80 API. 

A Standalone is templated over IN and OUT (the templates of the Driver). It is also templated over State and ExtendedState. Explanations:

### State

While IN is the input to a robot driver, State represents the desired state of *one* actuator of the robot.  For example, the State can be the position of a joint. 

State is a user developed class which should inheritate [o80::State](https://github.com/intelligent-soft-robots/o80/blob/vberenz/doc/include/o80/state.hpp).

Standalone is templated over State because o80 allows to send desired states command to each actuator independantly. The Standalone will receive commands related to desired States, and will convert them to instance of IN before forwarding them to the driver. The user implemented Standalone class will need to implement a [convert method](https://github.com/intelligent-soft-robots/o80/blob/master/include/o80/standalone.hpp#L155) for converting instances of State into instances of IN.

A State class does not only declare the desired state of a joint, but also [methods](https://github.com/intelligent-soft-robots/o80/blob/master/include/o80/state.hpp#L46) defining how desired states interpolates.

 This will allow for example o80 user API to support duration command method, which will request the desired state of an actuator to reach, starting from its current desired state, a target desired state of a specified duration (this API is presented in a later section, but [here](https://github.com/intelligent-soft-robots/o80_example/blob/master/demos/duration_commands.py#L24) a preview). 

The o80::State base class implements simple [linear interpolation methods](https://github.com/intelligent-soft-robots/o80/blob/master/include/o80/state.hxx) for basic types (int, double, float). If the user State encapsulate more complex data, and/or if the interpolation should not be linear, the "intermediate_state" methods should be overridden. 

If an actuator state consists of a boolean, [o80::BoolState](https://github.com/intelligent-soft-robots/o80/blob/master/include/o80/bool_state.hpp)  can be used.

If the actuator state is empty (i.e. the hardware is a sensor that takes no input), [o80::VoidState](https://github.com/intelligent-soft-robots/o80/blob/vberenz/doc/include/o80/void_state.hpp) can be used.

For example, the toy Joint class is a valid State class:

- [joint.hpp](https://github.com/intelligent-soft-robots/o80_example/blob/master/include/o80_example/joint.hpp)
- [joint.cpp](https://github.com/intelligent-soft-robots/o80_example/blob/master/src/joint.cpp)

### ExtendedState

The template OUT of a Driver is the arbitrary output of a robot.

A o80 [Observation](https://github.com/intelligent-soft-robots/o80/blob/master/include/o80/observation.hpp) output of an o80 Standalone is more structured, and has three parts:

- An instance of [States](https://github.com/intelligent-soft-robots/o80/blob/master/include/o80/states.hpp), which encapsulates the current desired state of each actuator
- An instance of [States](https://github.com/intelligent-soft-robots/o80/blob/master/include/o80/states.hpp), which encapsulates the current observed state of each actuator
- the extended state (an arbitrary user class)

During runtime, the o80 Standalone will call the [get](https://github.com/intelligent-soft-robots/o80/blob/vberenz/doc/include/o80/driver.hpp#L16) method of the driver and retrieve an instance of OUT. It will then need to "convert" this OUT instance into an instance of [o80::States](https://github.com/intelligent-soft-robots/o80/blob/master/include/o80/states.hpp). This instance of o80:States will encapsulate the current observed state of each actuator of the robot. The Standalone user class should implement this convert function.

It may be that the instance of OUT returned by the get function of the driver encapsulate data other than the observed state of the robot actuator. This information may be encapsulated in o80 Observation via the extended state. During runtime, the standalone instance will call its ["enrich_extended_state"]((https://github.com/intelligent-soft-robots/o80/blob/master/include/o80/states.hpp)) function to convert the instances of OUT into instances of ExtendedState. As OUT, EXTENDED_STATE is an arbitrary user class that templates Standalone. Note: OUT and EXTENDED_STATE may be the same class, in which case the "enrich_extended_state" function becomes trivial. If there are no sensory information beyond the actuators state, a [void extended state](https://github.com/intelligent-soft-robots/o80/blob/masterinclude/o80/void_extended_state.hpp) can be used.

### Summary

To implement an o80 Standalone, one must implement:

- a Driver class inherating [o80::Driver](https://github.com/intelligent-soft-robots/o80/blob/master/include/o80/driver.hpp). This class imlements the communication with the hardware.
- An OUT class and a out class that templates this driver class. Instances of these class are used by the driver to communicate with the hardware.
- A State class inherating [o80::State](https://github.com/intelligent-soft-robots/o80/blob/master/include/o80/state.hpp). An instance of State represente the state of an actuator.
- An ExtendedClass, which is an arbitrary class which will encapsulate robot's information other than states. Often, ExtendedClass can also be the OUT class.
- A Standalone, inherating [o80::Standalone](https://github.com/intelligent-soft-robots/o80/blob/master/include/o80/standalone.hpp) which is templated over the Driver class, the State class and the ExtendedClass.

## Step3 : library

The documentation above request to create 5 classes : In, Out, State, Driver and Standalone. Once these class declared, they shall be compiled in a new library that links with robot_interfaces and o80, as shown [here](https://github.com/intelligent-soft-robots/o80_example/blob/master/CMakeLists.txt#L30).

## Step4 : python bindings

[pybind11](https://pybind11.readthedocs.io/en/stable/) can be used to create python bindings over the library above.

o80 provides [helper functions](https://github.com/intelligent-soft-robots/o80/blob/master/include/o80/pybind11_helper.hpp) which will generate the bindings automatically. See the [example](https://github.com/intelligent-soft-robots/o80_example/blob/master/srcpy/wrappers.cpp).   
The macro for the creation of the pybind11 module need to be added to the [CMakeLists.txt](https://github.com/intelligent-soft-robots/o80_example/blob/master/CMakeLists.txt#L42).

The [o80::create_python_bindings](https://github.com/intelligent-soft-robots/o80/blob/e050f1ae16b47c4000f85fb237ded7835b3b3daa/include/o80/pybind11_helper.hpp#L73) function is templated over the Driver and the Standalone class, but other arguments (... DriverArgs). These correspond to the signature of the Driver's constructor. For example, this driver's constructor takes two double as arguments. Thus, the corresponding [create_python_bindings function](https://github.com/intelligent-soft-robots/o80_example/blob/c83398d19cd8bb69930fabe42b7cabf1a1d8fa38/srcpy/wrappers.cpp#L6) is templated over two doubles.

Note: important detail. The name of the pybind11 library compiled and binded in the CMakeLists.txt has to match the name of the library passed in the PYBIND11_MODULE macro (as in [here](https://github.com/intelligent-soft-robots/o80_example/blob/c83398d19cd8bb69930fabe42b7cabf1a1d8fa38/CMakeLists.txt#L45) and [here](https://github.com/intelligent-soft-robots/o80_example/blob/c83398d19cd8bb69930fabe42b7cabf1a1d8fa38/srcpy/wrappers.cpp#L4).

## Step5: compilation

For the moment, o80 and robot_interfaces supports only catkin and catkin tools, and the installation instruction should have resulting in the update of your catkin workspace (or the creation of a new catkin workspace), and therefore "catkin_make" or "catkin build" can be used for compilation.

Note that only python3 is supported for the bindings, and therefore the path to the correct python executable must be provided for compilation, for example:

```bash
catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3 install
```

