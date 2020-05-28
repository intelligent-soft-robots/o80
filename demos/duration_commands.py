import time
import o80
import o80_example
import o80_plotting

SEGMENT_ID = "o80_example"
FREQUENCY = 500
BURSTING_MODE = False
DRIVER_MIN = 0
DRIVER_MAX = 100
WINDOW = (800,800)

o80_example.start_standalone(SEGMENT_ID,
                             FREQUENCY,
                             BURSTING_MODE,
                             DRIVER_MIN,
                             DRIVER_MAX)

time.sleep(1.0)
#o80_plotting.start(o80_example,SEGMENT_ID,
#                   500,(DRIVER_MIN,DRIVER_MAX),WINDOW,FREQUENCY)

frontend = o80_example.FrontEnd(SEGMENT_ID)

class TrajectoryPoint:
    def __init__(self,duration,value1,value2):
        self.duration = int(duration*1e6)
        self.value1 = value1
        self.value2 = value2

trajectory = [ TrajectoryPoint(5,50,100),
               TrajectoryPoint(3,100,50),
               TrajectoryPoint(3,50,100),
               TrajectoryPoint(5,0,0) ]

state = o80_example.State()

state.set(100)
frontend.add_command(0,state,o80.Duration_us(2000000),o80.Mode.OVERWRITE)

def commented():
    for tp in trajectory:
        state.set(tp.value1)
        frontend.add_command(0,state,
                             o80.Duration_us(tp.duration),
                             o80.Mode.QUEUE)
        state.set(tp.value2)
        frontend.add_command(1,state,
                             o80.Duration_us(tp.duration),
                             o80.Mode.QUEUE)

#frontend.pulse_and_wait()

start = time.time()
while time.time()-start < 5:
    observation = frontend.pulse()
    desired = observation.get_desired_states()
    v1 = desired.get(0).get()
    print(observation.get_iteration(),"|",v1)
    time.sleep(0.5)

#o80_plotting.stop(SEGMENT_ID)

o80_example.stop_standalone(SEGMENT_ID)
    
