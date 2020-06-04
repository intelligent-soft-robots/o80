import time
import o80
import o80_example

SEGMENT_ID = "o80_example"

frontend = o80_example.FrontEnd(SEGMENT_ID)

# replaying the last 5000 iterations
# (i.e. last 5 seconds at 1000Hz backend frequency)
current_iteration = frontend.read().get_iteration()
target_iteration = current_iteration-5000
observations = frontend.get_observations_since(target_iteration)

print("replay from:",target_iteration)

# going to first values at reasonable speed
desired1 = observations[0].get_desired_states()
v1 = desired1.get(0).get()
v2 = desired1.get(1).get()
state = o80_example.State()
for dof,value in zip((0,1),(v1,v2)):
    state.set(value)
    frontend.add_command(dof,state,o80.Speed.per_second(500),o80.Mode.OVERWRITE)
frontend.pulse()

# replay
for observation in observations:
    desired = observation.get_desired_states()
    v1 = desired.get(0).get()
    v2 = desired.get(1).get()
    for dof,value in zip((0,1),(v1,v2)):
        state.set(value)
        frontend.add_command(dof,state,o80.Mode.QUEUE)
frontend.pulse_and_wait()



