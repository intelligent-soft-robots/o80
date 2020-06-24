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
