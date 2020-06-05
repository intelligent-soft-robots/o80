
Readme
======

# Overview

o80 (pronounced 'oh eighty') is a tool for synchronizing processes while organizing exchange of information between them.
The information exchanged are commands for computing (robotic) desired states and observations.

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

The "add_command" method of the frontend is flexible, for example:

```python

# the desired state will reach the value of 100  at 3 units per second
frontend.add_command(0,State(100),Speed.per_second(3),QUEUE)

# after this, the desired state will reach the value of 150 in 5000 robot
# control iterations
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

You may wonder what "State" corresponds to. It is arbitrary as State is a python wrapper over an arbitrary
c++ class. For example, it can be for an actuator of your robot the desired position and velocity.
o80 is templated, and can accept any arbirary code of "State".

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
    state0 = observed_states.get(0)
    simulation.add_command(0,state0,OVERWRITE)
    simulation.pulse()
    if observation.get_iteration()>=end_iteration:
        break

```

This is the gist of o80. For more concrete example, you may check: [demos](https://github.com/intelligent-soft-robots/o80_example/tree/master/demos).

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

For a detailed installation procedure that will end up with demos you will be able to run,
visit: [o80 example](https://github.com/intelligent-soft-robots/o80_example)

You may also look at these other projects that use o80:

- [o80 roboball2d](https://github.com/intelligent-soft-robots/o80_roboball2d)
- [o80 PAM](https://github.com/intelligent-soft-robots/o80_pam)




The expected domain of usage is robotics, and inside robotics, reinforcement learning.

oCommands sent to robotic processes encapsulate the data required for computing desired states,



##1. Overview

o80 (pronounced 'oh eighty') is a tool for synchronizing processes while organizing exchange of information between them.
The expected domain of usage is robotics, and inside robotics, reinforcement learning.

Commands sent to robotic processes encapsulate the data required for computing desired states,
and the robotics processes share observations (i.e. observed state, desired state and some misc data).
While background robotics processes correspond to realtime c++ code, the user interfaces interfacing with them 
provide a trivial python api.

The main entities of o80 are frontends, backends and standalones.
A frontend is a (python) interface that allows user code to send commands and read observations.
A backend is hosted by a (c++ realtime) process which uses the commands provided by the frontend to compute the
current desired states of each robot actuator and generate observations.
A standalone is the (c++ realtime) processes that hosts the backend, and interfacing it with the hardware (or simulation).

Typically, several frontends, backends and standalone will coexist. E.g. you may have backends connected to various sensors,
backends connected to various actuators, and backends running in simulated environment, all at the same time.
The (python) user code will then use the API provided by the correspondings frontends to synchronize all the related processes.
For example, it may be that via the frontends, the user code waits for a new sensor input to be generated and then use the
frontends to send related commands to the hardware and simulation backends running in parallel.
Using o80, creating via a python interface such architecture becomes trivial.

##2. We created it for machine learning research

For running a robotic experiment, typically a lots of devices have to be integrated (robot(s), simulation(s), sensor(s), logger(s), etc).
Typically, these devices runs at different frequences, and possibly needs to be synchronized one with another. On top of this,
because there are realtime constraints, this integration must typically support realtime c++.

On the other hand, machine learning researchers have low interest for such integration, which they may consider (to some extend) as a
some technical details time consuming to solve. What they typically would like to have access to is a (trivial) python API allowing them to
pass actions and read observations. And if possible, such API could be used in some convenient ML environment (e.g. gym).

The goal of o80 is to help researchers into creating such an API, which manages and hide the details of the integration.

##3. What o80 is not

o80 is used to have frontends sending commands that will have backends computing related desired states. It is assumed the robot which will
receive these desired states do have the (realtime) controllers required to compute the lower level commands that will have the real state of the robot
converging to these desired states. o80 is not a control architecture.

##4. Integration with your hardware 

To perform (c++) integration between o80 and your hardware, you need to program the base classes for state and observation. 
Then you need to program a driver class which follows this simple interface:
[driver interface](https://github.com/open-dynamic-robot-initiative/robot_interfaces/blob/master/include/robot_interfaces/robot_driver.hpp).

The 

Integration of o80 is 
o80 is a templated c++ library with python bindings.

