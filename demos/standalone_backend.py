import signal
import time
import o80
import o80_example
import o80_plotting

SEGMENT_ID = "o80_example"
FREQUENCY = 1000
BURSTING_MODE = False
DRIVER_MIN = 0.
DRIVER_MAX = 100.
WINDOW = (800,1200)

def run():

    o80.clear_shared_memory(SEGMENT_ID)
    
    running = True
    def _stop(signum,frame):
        running = False 
    signal.signal(signal.SIGINT,_stop)
    
    
    o80_example.start_standalone(SEGMENT_ID,
                                 FREQUENCY,
                                 BURSTING_MODE,
                                 DRIVER_MIN,
                                 DRIVER_MAX)
    
    o80_plotting.start(o80_example,SEGMENT_ID,
                       500,(DRIVER_MIN,DRIVER_MAX),WINDOW,FREQUENCY)

    while running:
        time.sleep(0.1)

    o80_plotting.stop(SEGMENT_ID)
    o80_example.stop_standalone(SEGMENT_ID)



if __name__ == "__main__":

    run()
