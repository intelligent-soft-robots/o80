import math
import time
import o80
import o80_example

SEGMENT_ID = "o80_example"

frontend = o80_example.FrontEnd(SEGMENT_ID)


initial_iteration = frontend.read().get_iteration()
start_iteration = initial_iteration+1000

state = o80_example.State()
state.set(50)
frontend.add_command(0,state,
                     o80.Iteration(start_iteration),
                     o80.Mode.OVERWRITE)
frontend.add_command(1,state,
                     o80.Iteration(start_iteration),
                     o80.Mode.OVERWRITE)


# joint 0: full resolution
value=0
increment = 0.001
amplitude = 30
for _ in range(15000):
    value += increment
    v0 = +amplitude*math.sin(value)+50
    state.set(v0)
    frontend.add_command(0,state,
                         o80.Mode.QUEUE)

# joint 1: reduced resolution (by factor 500)
value=0
value_increment = 0.5
amplitude = 30
target_iteration = start_iteration
iteration_increment = 500
for _ in range(30):
    target_iteration += iteration_increment
    value += value_increment
    v1 = +amplitude*math.sin(value)+50
    state.set(v1)
    frontend.add_command(1,state,
                         o80.Iteration(target_iteration),
                         o80.Mode.QUEUE)

state.set(0)
frontend.add_command(0,state,
                     o80.Duration_us.seconds(1),
                     o80.Mode.QUEUE)
frontend.add_command(1,state,
                     o80.Duration_us.seconds(1),
                     o80.Mode.QUEUE)
frontend.pulse();


