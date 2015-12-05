import logging
import colorlog # colors log output
from vrep_bridge import vrep_bridge # for getState, setState
from lulu_pcol_sim import sim
import sys # for argv, stdout

class Kilobot():

    """Class used to store the state of a Kilobot robot for use in a controller that used Pcolonies."""

    def __init__(self, pcolony):
        self.colony = pcolony # reference to the Pcolony used to control this robot
        self.raw_input_state = {} # dictionary of raw sensor values
        self.output_state = {
                "motion" : vrep_bridge.Motion.stop, # motion (vrep_bridge.Motion) motion type
                "rgb_led" : [0, 0, 0] # light [r, g, b] with values between 0-2
                } # dictionary of output states
    # end __init__()

    def procInputModule(self):
        """Process raw_state info received from sensors and populate the input module agents with significant objects"""
        pass
    # end procInputModule()

    def procOutputModule(self):
        """Process the objects present in the output module agents and transform them into commands that can be sent to the kilobot through vrep_bridge.setState()"""
        self.output_state["motion"] = vrep_bridge.Motion.stop # default motion
        self.output_state["rgb_led"] = [0, 0, 0] # default color (off)

        if ('m_S' in colony.agents['AG_motion'].obj):
            self.output_state["motion"] = vrep_bridge.Motion.forward
        elif ('m_L' in colony.agents['AG_motion'].obj):
            self.output_state["motion"] = vrep_bridge.Motion.left
        elif ('m_R' in colony.agents['AG_motion'].obj):
            self.output_state["motion"] = vrep_bridge.Motion.right

    #end procOutputModule()

# end class Kilobot


##########################################################################
#   MAIN
formatter = colorlog.ColoredFormatter(
        "%(log_color)s%(levelname)-8s %(message)s %(reset)s",
        datefmt=None,
        reset=True,
        log_colors={
                'DEBUG':    'cyan',
                'INFO':     'green',
                'WARNING':  'yellow',
                'ERROR':    'red',
                'CRITICAL': 'red,bg_white',
        },
        secondary_log_colors={},
        style='%'
)
if ('--debug' in sys.argv):
    colorlog.basicConfig(stream = sys.stdout, level = logging.DEBUG)
else:
    colorlog.basicConfig(stream = sys.stdout, level = logging.INFO) # default log level

stream = colorlog.root.handlers[0]
stream.setFormatter(formatter);

if (len(sys.argv) < 2):
    logging.error("Expected input file path as parameter")
    exit(1)

# read Pcolony from file
colony = sim.readInputFile(sys.argv[1])
# make link with v-rep
bridge = vrep_bridge.VrepBridge()

robot = Kilobot(colony)

while (True):
    print("\n")
    robot.raw_input_state = bridge.getState()
    robot.procInputModule()
    
    sim_result = colony.runSimulationStep()
    # if the simmulation result is other than step finished (i.e. no_more_exec or error)
    if (sim_result != sim.SimStepResult.finished):
        # exit the loop
        logging.warn("Exiting loop")
        break

    robot.procOutputModule()
    bridge.setState(robot.output_state["motion"], robot.output_state["rgb_led"])
