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

# Reading observations

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


## next observation

```python
frontend.reset_next_index()
observation1 = frontend.wait_for_next()
iteration1 = observation1.get_iteration()
observation2 = frontend.wait_for_next()
# iteration2 will be iteration1+1
iteration2 = observation2.get_iteration()
```

The method wait_for_next returns the "next" observation since the previous call to wait_for_next, or wait for such observation to be generated (by the standalone process).

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


# Using several frontends

Several frontends may connect simultaneously to the same segment_id. This is useful, for example, to create a logging process. For example, in one python executable one may send commands, while in another independant execuble, one may log information related to observations. For example:

```python
while True:
    observation = frontend.wait_for_next()
    print(observation.display())
    time.sleep(0.01)
```

*Important* : while several instances of FrontEnd may run concurrently, only one of them should by used to send commands. Sending commands via several FrontEnds may have unexpected effects. 

## Putting things together

Using the API described above, it is possible for example:

- to generate in python a full trajectory whic specifies a desired state for each Standalone iteration ([code](https://github.com/intelligent-soft-robots/o80_example/blob/master/demos/full_trajectory.py#L20))

- to have a robot replaying another robot actions ([code](https://github.com/intelligent-soft-robots/o80_example/blob/master/demos/delay.py))

- to have a robot mirroring another robot ([code](https://github.com/intelligent-soft-robots/o80_example/blob/master/demos/mirroring.py))
