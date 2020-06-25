
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


