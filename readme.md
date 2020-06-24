
Readme
======

# Overview

o80 (pronounced 'oh eighty') is a tool for synchronizing processes while organizing exchange of information between them.
The information exchanged are commands for computing (robotic) desired states and observations, where state and observation are user class (and o80 classes are templated over them).

Here is a basic usage of o80, using python:

```python

# starts a process running robot control at
# 1000Hz
frequency = 1000
start_standalone("my_robot",frequency)

# creating a frontend for communicating with the robot
frontend = FrontEnd("my_robot")

# creating a command requesting the desired state of
# the 0th actuator to reach the value of 100 in 3 seconds
frontend.add_command(0,State(100),Duration.seconds(3),QUEUE)
# same for the 1th actuator
frontend.add_command(1,State(100),Duration.seconds(3),QUEUE)

# send the commands. The desired states of 0th and 1st dof
# of the robot interpolates from their initial values to 100.
# Once this is achieved, an observation is returned
observation = frontend.pulse_and_wait()

# reading the state of the robot 
state0 = observation.get_observed_states().get(0).get()
state1 = observation.get_observed_states().get(1).get()

# stopping the robot
stop_standalone("my_robot",frequency)

```

In the example above, the standalone process spawns a realtime process corresponding
to c++ (realtime) code. The frontend communicates under the hood with this process via
a realtime shared memory.

The frontend provides the user with an "add_command" which is flexibe (i.e. has many convenient overloads). For Example:

```python

# the desired state will reach the value of 100 at 3 units per second
frontend.add_command(0,State(100),Speed.per_second(3),QUEUE)

# after this, the desired state will reach the value of 150 at the 5000th 
# robot's control iteration
frontend.add_command(0,State(150),Iteration(5000),QUEUE)

# the desired states will increase by 5 for each of the following iteration
frontend.add_command(0,State(155),QUEUE)
frontend.add_command(0,State(160),QUEUE)
frontend.add_command(0,State(170),QUEUE)

# sending the commands to the robot and returning immediately
frontend.pulse()

# letting the robot run for 1 second, then cancelling all running
# commands and requesting the desired state to decrease to 0 in
# 5 seconds
time.sleep(1.0)
frontend.add_command(0,State(0),Duration.seconds(5),OVERWRITE)
frontend.pulse()

```

o80 classes are templated over State. In the above, State is a python wrapper over an arbitrary
c++ "State" class. For example, it can be for an actuator of your robot the desired position and velocity.

o80 can also be used to synchronize processes. Here an example that has a simulated robot mirroring a
a real robot:

```python

robot = Frontend("real_robot")
simulation = Frontend("simulation")

# sending a trajectory to one of the
# robot actuator
starting_iteration = robot.read().get_iteration()
end_iteration = starting_iteration+len(values)
for value in values:
    robot.add_command(0,State(value),QUEUE)
robot.pulse()

# having the simulated robot mirroring the real robot
while True:
    observation = robot.wait_for_next()
    observed_states = observation.get_observed_states()
    for actuator in range(nb_actuators):
        state = observed_states.get(actuator)
        simulation.add_command(actuator,state,OVERWRITE)
    simulation.pulse()
    if observation.get_iteration()>=end_iteration:
        break

```

This is the gist of o80. For a fully commented concrete example / tutorial, you may check: [demos](https://github.com/intelligent-soft-robots/o80_example/tree/master/demos).

# Installation

o80 is a c++ library with python wrappers. It uses catkin for compilation.
It has been tested only on ubuntu 18.04 and python3.
To use o80, you will need:
- a machine installed with ubuntu 18.04, possibly with a preempt patch kernel, if your robot requires it.
- installing a few dependencies (apt and pip installation only)
- creating a catkin workspace with o80 + dependencies packages (we use the project manager [Treep](https://pypi.org/project/treep/) to make this trivial)
- creating a catkin project that will host your c++ code. Your c++ code will be the classes used for State, Observation, as well as the Driver interfacing
  o80 and your robot. o80 has been designed to that this code may be minimalist.
- create the python binders (o80 provides some high level functions to makes this trivial)

o80 is mostly a templated header library. It can thus only be installed in relation with a package which provides concrete class to be used as template. Thus the detailed installation procedure (that end up with demos you will be able to run) is available along with a package providing such concrete classe:  [o80 example](https://github.com/intelligent-soft-robots/o80_example)

- [o80 roboball2d](https://github.com/intelligent-soft-robots/o80_roboball2d)
- [o80 PAM](https://github.com/intelligent-soft-robots/o80_pam)

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

# Usage

Once the catkin workspace sourced, o80 may be used via its python API. Concrete examples of usage of this API is provided in the [o80 example](https://github.com/intelligent-soft-robots/o80_example) package.

The code snippets below assumes the python binded library is called "o80_robot", and that the State class encapsulates only one integer. 

## Starting a standalone

```python
import o80_robot

segment_id = "id1"
frequency = 1000 # in Hz
bursting_mode = False
driver_arg = [1,1]

o80_robot.start_standalone(segment_id,
                           frequency,
                           bursting_mode,
			   *driver_args)
```

The above starts an o80 Standalone that will run at 1000Hz, i.e. it will run 1000 iterations per second, and at each iteration will call the set() and get() method of the encapsulated driver.
The argument is an arbitrary id (segment_id), the frequency, the bursting mode (later explained, set to False in doubt) and the arguments required for instantiating the driver.
The standalone is spawned in a separated (c++) thread.

## Starting a frontend

```python
import o80
import o80_robot

segment_id = "id1"

frontend = o80_robot.FrontEnd(segment_id)
```

This starts a frontend, i.e. the instance of an object able to communicate with the standalone (i.e. send commands, read observation). The segment_id passed should be the same as the one used when starting the standalone.

**The frontend and the standalone can be instantiated in two different scripts**. Just: the standalone must be started first. The frontend and the standalone will communicate under the hood via a realtime shared memory. 

## Buffering commands

```python
target_value = 1000
# actuator 0
frontend.add_command(0,o80_robot.State(target_value),o80.Mode.OVERWRITE)
# actuator 1
frontend.add_command(1,o80_robot.State(target_value),o80.Mode.OVERWRITE)
```

The above creates a command requesting the desired state of actuator 0 to take the value 1000 as soon as possible (o80.Mode.OVERWRITE explained later). 
It also creates a command requesting the same desired state value of actuator 1.
These commands are not applied yet, they are just buffered in the frontend.

## Sending commands

```python
frontend.pulse()
```

The buffer of command is purged, and all the commands are sent to the Standalone which execute them. Consequently, the desired state of actuators 0 and 1 get to the value 1000.

## Time, Speed and Iteration commands

The commands passed above requested the desired states of the actuators to change immediately upon execution of the command.
One may request instead for the desired states to interpolate from their current value to their target value over several iterations.
Several overload of the add_command function may be used:

### Duration commands

```python
actuator = 0
target_value = 1000
frontend.add_command(actuator,o80_robot.State(target_value),o80.Duration.milliseconds(2000),o80.Mode.OVERWRITE)
```

The command above requests the desired state value to interpolate from its current value to the target value over two seconds.

### Speed commands

```python
actuator = 0
target_value = 1000
frontend.add_command(actuator,o80_robot.State(target_value),o80.Speed.per_millisecond(1),o80.Mode.OVERWRITE)
```

The command above requests the desired state value to interpolate from its current value to the target value at the speed of 1 unit per milliseconds.

### Iteration commands

```python
actuator = 0
target_value = 1000
frontend.add_command(actuator,o80_robot.State(target_value),
                     o80.Iteration(5000),o80.Mode.OVERWRITE)
```

The command above request to interpolate from its current value so that reaching the target value at the 5000th iteration of the standalone. 
Note that it does **not** mean reaching the target value over 5000 iterations. When the standalone is started, it starts iterating at its specified frequency, and keeps count of its iteration number (starting at 0). If the current iteration number is 10 when the command is started, the desired state will iterate over 4990 iteration. If the iteration number is 4950, it will interpolate over 50 iterations.

To specify the number of iteration to interpolate over:

```python
actuator = 0
target_value = 1000
relative = True
reset = True
frontend.add_command(actuator,o80_robot.State(target_value),
                     o80.Iteration(5000,relative,reset),o80.Mode.OVERWRITE)
```

The above request to reach to target value at the 5000th *relative* iteration number, i.e. the iteration number relative to the last command for which "reset" was True was started.
In this case, this command reset the iteration count, and then considers iteration number relative to this resetted number. It will thus request to interpolate over 5000 iterations.

## Interrupting command

```python
actuator = 0
target_value = 1000
frontend.add_command(actuator,o80_robot.State(target_value),
                     o80.Duration.milliseconds(2000),o80.Mode.OVERWRITE)
frontend.pulse()
time.sleep(1)
target_value = 500
frontend.add_command(actuator,o80_robot.State(target_value),
                     o80.Duration.milliseconds(2000),o80.Mode.OVERWRITE)
frontend.pulse()
```

The above sends a duration command to the standalone, which starts to execute it. 
But after 1 seconds, the frontend sends another command (note: "pulse" returns immediately, not upon finalized execution of the command). 

Because this second command is sent using the mode "o80.Mode.OVERWRITE", it requests the cancelation of the current command. The standalone interrupt its running command and starts execution of the second command.

This has a different behavior:

```python
actuator = 0
target_value = 1000
frontend.add_command(actuator,o80_robot.State(target_value),
                     o80.Duration.milliseconds(2000),o80.Mode.OVERWRITE)
frontend.pulse()
time.sleep(1)
target_value = 500
frontend.add_command(actuator,o80_robot.State(target_value),
                     o80.Duration.milliseconds(2000),o80.Mode.QUEUE)
frontend.pulse()
```

The 'QUEUE' mode is used, thus the standalone finishes the first command, then starts execution of the second command.

Note that this is equivalent to:

```python
actuator = 0
target_value = 1000
frontend.add_command(actuator,o80_robot.State(target_value),
                     o80.Duration.milliseconds(2000),o80.Mode.OVERWRITE)
target_value = 500
frontend.add_command(actuator,o80_robot.State(target_value),
                     o80.Duration.milliseconds(2000),o80.Mode.QUEUE)
frontend.pulse()
```

## Blocking pulse

```python
actuator = 0
target_value = 1000
frontend.add_command(actuator,o80_robot.State(target_value),o80.Duration.milliseconds(2000),o80.Mode.OVERWRITE)
target_value = 500
frontend.add_command(actuator,o80_robot.State(target_value),o80.Duration.milliseconds(2000),o80.Mode.QUEUE)
frontend.pulse()
```

In the above, the pulse method returns immediately, which allows for the frontend to buffer new commands while other commands are executed.

Alternatively, to block the script until these two commands are completed:

```python
actuator = 0
target_value = 1000
frontend.add_command(actuator,o80_robot.State(target_value),o80.Duration.milliseconds(2000),o80.Mode.OVERWRITE)
target_value = 500
frontend.add_command(actuator,o80_robot.State(target_value),o80.Duration.milliseconds(2000),o80.Mode.QUEUE)
frontend.pulse_and_wait() # !
```

Instead of returning once the commands have been finalized, one may return at a specific standalone iteration number

```python
frontend.pulse(o80.Iteration(5000)) 
```

This function will block until the 5000th iteration of the standalone has been reached.

# Reading observation

## latest observation

The pulse and pulse_and_wait method do not only purge the command buffer, but they also return an observation.

```python
observation = frontend.pulse()
```

Alternatively, the latest method also returns an observation (but does not purge the command buffer)

```python
observation = frontend.latest()
```

Observation encapsulate these data:

```python
observed_states = observation.get_observed_states()
desired_states = observation.get_observed_state()
iteration_number = observation.iteration()
frequency = observation.get_frequency()
extended = observation.get_extended_state()
```

observed_states and desired_states are instances of States. To get the State of the 0th actuator:

```python
state = observed_states.get(0)
value = state.get()
```

*iteration number* is a integer corresponding to the standalone iteration number at the time this observation was created.

*frequency* is the frequency of the standalone as observed at the corresponding iteration. 

*extended* is an instance of the "extended class" templating the Standalone. 


## observation history

```python
history = frontend.get_latest_observations(1000)
```

History is a list of instances of Observation of size 1000, corresponding to the last 1000 observations writen by the standalone (one per iteration).

Alternatively:
 
```python
starting_iteration = frontend.latest().get_iteration()
actuator = 0
target_value = 1000
frontend.add_command(actuator,o80_robot.State(target_value),o80.Duration.milliseconds(2000),o80.Mode.OVERWRITE)
target_value = 500
frontend.add_command(actuator,o80_robot.State(target_value),o80.Duration.milliseconds(2000),o80.Mode.QUEUE)
frontend.pulse_and_wait() 
history = frontend.get_observation_since(starting_iteration)
```

## Putting things together

Using the API described above, it is possible for example:

- to generate in python a full trajectory whic specifies a desired state for each Standalone iteration ([code](https://github.com/intelligent-soft-robots/o80_example/blob/master/demos/full_trajectory.py#L20))

- to have a robot replaying another robot actions ([code](https://github.com/intelligent-soft-robots/o80_example/blob/master/demos/delay.py))

- to have a robot mirroring another robot ([code](https://github.com/intelligent-soft-robots/o80_example/blob/master/demos/mirroring.py))


# Bursting mode

## Overview

It is possible to start the backend in bursting mode.

```python
import o80_robot

segment_id = "id1"
frequency = -1 # in Hz
bursting_mode = True # !
driver_arg = [1,1]

o80_robot.start_standalone(segment_id,
                           frequency,
                           bursting_mode,
			   *driver_args)
```

When *not* using the bursting mode, the start_standalone function spaws a process that will iterate (i.e interact with the driver) at the specified frequency.

When using the bursting mode, the specified frequency is ignored. Instead, the process will iterate only when a frontend requests it to do so.

```python
frontend = o80_robot.FrontEnd(segment_id)
frontend.burst(10)
```

The code above requests the standalone process to perform exactly 10 iterations, and then wait. The burst function returns after the 10 iterations have been perfomed. The 10 iterations are performed as fast as the driver software allows.

Similarly to the functionos latest, pulse and pulse_and_wait; the burst method returns an observation.

```python
current_iteration = frontend.latest().get_iteration()
observation = frontend.burst(10)
# new_iteration is current_iteration+10
new_iteration = observation.get_iteration()
```

The burst function also send the current list of commands.

```python
target_value = 1000
frontend.add_command(0,o80_robot.State(target_value),
                     o80.Iteration(2000,True,True),
                     o80.Mode.OVERWRITE)
observation = frontend.burst(2000)
# value will be 1000
value = observation.get_desired_states().get(0).get()
```

## Usage

The typical usage of the bursting mode is interaction with a simulator. Via the frontend, it is possible to create commands (or queues of commands), and then having them executed as fast the simulator allows.

Another usage is synchronization between process:

```python
obs1 = frontend1.burst(3)
frontend2.add_command(0,
                      obs1.get_observed_value().get(0),
		      o80.Mode.OVERWRITE)
frontend2.burst(1)
```


# Integration in user software

So far, we documented the usage of o80 via a standalone, i.e. a python method which spaws a process running the (c++) interaction with the driver.

But one may desire to use o80's frontend API while using their own software architecture.

It is possible to do so by encapsulating an instance of *o80::BackEnd* to the architecture. This could be done either in python and in c++.

## in python

### backend

Similarly than for the FrontEnd and the start_standalone method, the python package wraps the BackEnd class.

```python
import o80_robot
backend = o80_robot.BackEnd(segment_id)
```

A backend instance provides a single method, pulse, which takes the current observed states of the joints as arguments, and returns the current desired states to apply to the robot (as computed based on the queues of commands sent by an eventual frontend).

A call to BackEnd::pulse performs:

- read the queue of commands as generated by an eventual frontend.
- compute for each joint its desired states, according to the current queue of commands.
- creates an instance of o80::Observation, and writes it to the shared memory (i.e. makes it available to FrontEnd::pulse, FrontEnd::latest, etc)
- returns an instance of o80::States, which encapsulate for each joint its desired state.

An instance of BackEnd may be used in the user's code:

```python

import o80_robot

#
# user initialization code
#
# ...
# 
backend = o80_robot.BackEnd(segment_id)

while running:

     # user code compute and extract from sensors
     # the current joint states

     # calling the pulse method of the backend
     # (current_states: list of size number of joints)
     desired_states = backend.pulse(current_states)

     # user code for applying the desired state
     # to the robot
     
     time.sleep(0.001)

```

The pulse function has several overloads. The most complete is:

```python


time_stamp = o80::time_now()
extended_state = MyUserClass()
iteration_update = False
current_iteration = 1000

frontend.pulse(time_stamp,
               current_states,
	       extended_state,
	       iteration_update,
	       1000)

```

- time_stamp will be the time stamp encapsulated in the Observation the BackEnd will write in the shared memory.
- extended_state is an instance of ExtendedState
- iteration_update to False means the BackEnd should not use its internal counter to set the iteration number of the Observation, but rather use the current_iteration argument (iteration_update to False and current_iteration to -1 to use the internal counter)


Other overloads are:

```python

# will use current time as time_stamp, default constructor of ExtendedState, internal iteration counter, and default constructors for desired states.
frontend.pulse()

# same as above, but will use the passed argument for extended state
frontend.pulse(extended_state)

# same as above, but will use the passed argument for desired states
frontend.pulse(desired_states)

```

### bursting mode

To use the bursting mode in a user software, you may update the control loop:

```python

import o80_robot
import o80

#
# user initialization code
#
# ...
# 
backend = o80_robot.BackEnd(segment_id)
burster = o80.Burster(segment_id)

while running:

     # user code compute and extract from sensors
     # the current joint states

     # calling the pulse method of the backend
     # (current_states: list of size number of joints)
     desired_states = backend.pulse(current_states)

     # user code for applying the desired state
     # to the robot

     # ! commented, frequency will be managed by
     # the burster instead
     # time.sleep(0.001)

     burster.pulse()

```

In the above, when its method pulse is called, the loop will hold until a frontend calls its burst function. 

## in c++

An equivalent API is provided in c++.


# Troubleshooting

## Processes hanging

### clearing the shared memory

A recurring issue when using o80 is that processes may hang if trying to connect to a deprecated shared memory. A deprecated shared memory occurs when a process is not cleanly exited (i.e. the destructor methods are not properly called).

This can be solved by called the o80::clear_shared_memory method using the suitable segment_id as argument.

o80::clear_shared_memory has not down side, and it is even good manner to call this function before a new standalone is instantiated.

```python

segment_id = "id0"
o80.clear_shared_memory(segment_id)
o80_robot.start_standalone(segment_id, ...)

```

If the clear_shared_memory does not work, you may try to delete the content of the folder /dev/shm. Note that this may also delete shared memory files used by process not related to o80.

### instantiating in the right order

A FrontEnd targeting a segment_id must always be started *after* a corresponding standalone or BackEnd has been instantiated. If the standalone or BackEnd is destroyed before the FrontEnd, the instance of FrontEnd will hang.




