import time
import o80
import o80_example

SEGMENT_ID = "o80_example"

frontend = o80_example.FrontEnd(SEGMENT_ID)

class TrajectoryPoint:
    def __init__(self,duration,value):
        self.duration = duration
        self.value = value

trajectory = [ TrajectoryPoint(250,60),
               TrajectoryPoint(250,50),
               TrajectoryPoint(250,70),
               TrajectoryPoint(250,50),
               TrajectoryPoint(250,80),
               TrajectoryPoint(250,50),
               TrajectoryPoint(250,90),
               TrajectoryPoint(250,50),
               TrajectoryPoint(250,100),
               TrajectoryPoint(250,0) ]

state = o80_example.State()

# playing a trajectory on first joint
starting_iteration = frontend.read().get_iteration()
for tp in trajectory:
    state.set(tp.value)
    frontend.add_command(0,state,
                         o80.Duration_us.milliseconds(tp.duration),
                         o80.Mode.QUEUE)
frontend.pulse_and_wait()

# replaying the trajectory of the first joint
# on the second joint with 2 seconds shift
observations = frontend.get_observations_since(starting_iteration)
time.sleep(2)
for observation in observations:
    joint1_state = observation.get_desired_states().get(0)
    frontend.add_command(1,joint1_state,
                         o80.Mode.QUEUE)
frontend.pulse_and_wait()



