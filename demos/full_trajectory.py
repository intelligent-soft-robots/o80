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


trajectory = [ TrajectoryPoint(10,50,50),
               TrajectoryPoint(10,100,0),
               TrajectoryPoint(100,50,50),
               TrajectoryPoint(10,80,20),
               TrajectoryPoint(20,20,80),
               TrajectoryPoint(30,0,0) ]


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


