import time
import fyplot
import o80
import o80_example

SEGMENT_ID = "o80_example"
FREQUENCY = 1000
BURSTING_MODE = False
DRIVER_MIN = 0.
DRIVER_MAX = 100.
WINDOW = (800,1200)


def _get_plot(frontend):

    plt = fyplot.Plot(SEGMENT_ID,50,WINDOW)
    
    def get_joint1():
        return frontend.read().get_desired_states().get(0).get()
    
    def get_joint2():
        return frontend.read().get_desired_states().get(1).get()

    def get_frequency():
        return frontend.read().get_frequency()
    
    desired_states_plot = ( (get_joint1,(0,255,0)),
                            (get_joint2,(255,0,0)) )
    frequency_plot = ((get_frequency,(255,0,0)),)
    plt.add_subplot((DRIVER_MIN,DRIVER_MAX),1000,desired_states_plot)
    plt.add_subplot((0,FREQUENCY+100),1000,frequency_plot)
    
    return plt


def run():

    o80.clear_shared_memory(SEGMENT_ID)
    
    o80_example.start_standalone(SEGMENT_ID,
                                 FREQUENCY,
                                 BURSTING_MODE,
                                 DRIVER_MIN,
                                 DRIVER_MAX)
    
    frontend = o80_example.FrontEnd(SEGMENT_ID)

    plot = _get_plot(frontend)
    plot.start()
    
    try :
        while True:
            time.sleep(0.1)
    except KeyboardInterrupt:
        pass

    plot.stop()
    o80_example.stop_standalone(SEGMENT_ID)

if __name__ == "__main__":
    run()
