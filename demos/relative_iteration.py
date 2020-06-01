import time
import o80
import o80_example
import o80_plotting

SEGMENT_ID = "o80_example"

frontend = o80_example.FrontEnd(SEGMENT_ID)

class TrajectoryPoint:
    def __init__(self,iteration,value1,value2):
        self.iteration = iteration
        self.value1 = value1
        self.value2 = value2


trajectory = [ TrajectoryPoint(500,50,50),
               TrajectoryPoint(1000,60,40),
               TrajectoryPoint(1500,40,60),
               TrajectoryPoint(2000,0,0) ]

state = o80_example.State()
relative = True
for _ in range(2):
    first_iter = True
    for tp in trajectory:
        state.set(tp.value1)
        iteration = o80.Iteration(tp.iteration,relative)
        if first_iter:
            iteration.reset()
            first_iter = False
        frontend.add_command(0,state,
                             iteration,
                             o80.Mode.QUEUE)
        state.set(tp.value2)
        iteration = o80.Iteration(tp.iteration,relative)
        frontend.add_command(1,state,
                             iteration,
                             o80.Mode.QUEUE)

frontend.pulse()


