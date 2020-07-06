
o80 (pronounced 'oh eighty') is a tool for synchronizing processes while organizing exchange of information between them.
The information exchanged are commands for computing (robotic) desired states and observations, where state and observation are user class (and o80 classes are templated over them).
o80 is in c++, with python wrappers.

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

A useful complement to this webpage is the documentation of the [o80_example](https://intelligent-soft-robots.github.io/code_documentation/o80/docs/sphinx/html/index.html) package, which provides a concrete implementation of an o80 project.

The github source code of o80 is [here](https://github.com/intelligent-soft-robots/o80).


