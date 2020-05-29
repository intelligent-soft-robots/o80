import time
import o80
import o80_example
import o80_plotting

SEGMENT_ID = "o80_example"

frontend = o80_example.FrontEnd(SEGMENT_ID)

class TrajectoryPoint:
    def __init__(self,speed,value1,value2):
        self.speed = speed
        self.value1 = value1
        self.value2 = value2


# going to initial state
#state = o80_example.State()
#state.set(50);
#frontend.add_command(0,state,o80.Duration_us(int(2*1e6)),o80.Mode.OVERWRITE)
#frontend.add_command(1,state,o80.Duration_us(int(2*1e6)),o80.Mode.OVERWRITE)
#frontend.pulse_and_wait()



trajectory = [TrajectoryPoint(5,100,0) ]
        
#trajectory = [ TrajectoryPoint(5,100,0),
#               TrajectoryPoint(10,0,50),
#               TrajectoryPoint(10,100,0),
#               TrajectoryPoint(5,0,0) ]


state = o80_example.State()
for tp in trajectory:
    state.set(tp.value1)
    frontend.add_command(0,state,
                         o80.Speed.per_second(tp.speed),
                         o80.Mode.QUEUE)
    state.set(tp.value2)
    frontend.add_command(1,state,
                         o80.Speed.per_second(tp.speed),
                         o80.Mode.QUEUE)

frontend.pulse_and_wait()


