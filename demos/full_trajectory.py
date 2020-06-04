import math
import time
import o80
import o80_example

SEGMENT_ID = "o80_example"

frontend = o80_example.FrontEnd(SEGMENT_ID)


state = o80_example.State()
state.set(50)
frontend.add_command(0,state,
                     o80.Duration_us.seconds(1),
                     o80.Mode.OVERWRITE)
frontend.add_command(1,state,
                     o80.Duration_us.seconds(1),
                     o80.Mode.OVERWRITE)

value=0
increment = 0.003
amplitude = 30
for _ in range(5000):
    value += increment
    v0 = +amplitude*math.sin(value)+50
    v1 = -amplitude*math.sin(value)+50
    state.set(v0)
    frontend.add_command(0,state,
                         o80.Mode.QUEUE)
    state.set(v1)
    frontend.add_command(1,state,
                         o80.Mode.QUEUE)

state.set(0)
frontend.add_command(0,state,
                     o80.Duration_us.seconds(1),
                     o80.Mode.QUEUE)
frontend.add_command(1,state,
                     o80.Duration_us.seconds(1),
                     o80.Mode.QUEUE)
frontend.pulse_and_wait()


