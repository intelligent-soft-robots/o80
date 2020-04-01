import o80_example as o80
import sys,time


def run(segment_id,frequency,bursting_mode):

    max_ = 1000
    min_ = 0
    
    o80.start_standalone(segment_id,
                         frequency,
                         bursting_mode,
                         float("+inf"),
                         float("+inf"),
                         min_,
                         max_)

    frontend = o80.FrontEnd(segment_id)

    while (o80.standalone_is_running(segment_id)):

        observation = frontend.read()

        print("iteration: "+str(observation.get_iteration())
              +"\tfrequency: "+str(observation.get_frequency()))

        time.sleep(1)

        
def stop(segment_id):

    o80.please_stop(segment_id)
        
    
def usage():

    print("\nusage:\n")
    print("\tpython standalone.py <segment_id>")
    print("\tpython standalone.py <segment_id> <frequency>")
    print("\tpython standalone.py <segment_id> bursting")
    print("\tpython standalone.py <segment_id> <frequency> bursting")
    print("\tpython standalone.py <segment_id> stop")
    print("\n")
    
def start(args):
    
    frequency = 2000
    bursting = False

    if(len(args)<1):
        usage()
        return
    
    segment_id = args[0]

    if segment_id == "stop":
        usage()
        return
    
    if "stop" in args[1:]:
        stop(segment_id)
        return
    
    if "bursting" in args[1:]:
        bursting = True

    for arg in args[1:]:
        try :
            frequency = int(arg)
        except :
            pass

    if bursting:
        bursting_msg = " in bursting mode"
    else :
        bursting_msg = " in non-bursting mode"
        
    print("\n* starting o80 standalone with segment id "+str(segment_id)+
          " frequency "+str(frequency)+
          bursting_msg+" *\n")

    run(segment_id,frequency,bursting)
    
if __name__== "__main__":

    args = sys.argv[1:]
    start(args)
