import time
import o80
import o80_example

SEGMENT_ID = "o80_example"

frontend = o80_example.FrontEnd(SEGMENT_ID)

class TrajectoryPoint:
    def __init__(self,duration,value1,value2):
        self.duration = duration
        self.value1 = value1
        self.value2 = value2


# sending a trajectory to first joint
trajectory = [ TrajectoryPoint(1000,100,0),
               TrajectoryPoint(500,0,50),
               TrajectoryPoint(500,100,0),
               TrajectoryPoint(1000,0,0) ]
state = o80_example.State()
for tp in trajectory:
    state.set(tp.value1)
    frontend.add_command(0,state,
                         o80.Duration_us.milliseconds(tp.duration),
                         o80.Mode.QUEUE)
frontend.pulse()

# having the second joint mirroring
# the value of half the first
time_start = time.time()
while time.time()-time_start < 3:
    obs = frontend.read()
    value = obs.get_desired_states().get(0).get()/2.0
    state.set(value)
    frontend.add_command(1,state,
                         o80.Mode.OVERWRITE)
    frontend.pulse()
    time.sleep(0.001)
    
    
