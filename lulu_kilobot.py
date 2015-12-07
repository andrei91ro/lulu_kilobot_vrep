import logging
import colorlog # colors log output
from vrep_bridge import vrep_bridge # for getState, setState
from lulu_pcol_sim import sim
import sys # for argv, stdout
from copy import deepcopy # for deepcopy (value not reference as = does for objects)

class Kilobot():

    """Class used to store the state of a Kilobot robot for use in a controller that used Pcolonies."""

    def __init__(self, uid, pcolony):
        self.uid = uid # the unique id of the robot 
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

        if ('m_S' in self.colony.agents['AG_motion'].obj):
            self.output_state["motion"] = vrep_bridge.Motion.forward
        elif ('m_L' in self.colony.agents['AG_motion'].obj):
            self.output_state["motion"] = vrep_bridge.Motion.left
        elif ('m_R' in self.colony.agents['AG_motion'].obj):
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
pObj = sim.readInputFile(sys.argv[1])
# make link with v-rep
bridge = vrep_bridge.VrepBridge()

if (type(pObj) == sim.Pcolony):
    robot = Kilobot(0, pObj)
else:
    robots = [] # array of Kilobot objects
    nrRobots = 3
    aloc = {"pi_minus" : 3}

    # the first robot uses the original Pcolony from the Pswarm
    robots.append(Kilobot(0, pObj.colonies["pi_minus"]))

    bridge.spawnRobots(nr = nrRobots - 1)
    # create aditional copies of the Pcolony and assign then to each robot
    for i in range(1, nrRobots):
        logging.debug("Creating colony for robot %d" % i)
        # add a new Pcolony name (with uid appended)
        pObj.C.append("pi_minus_" + str(i))
        logging.debug("pObj.C = %s" % pObj.C)
        # create a value copy of the colony and store it under the new name
        pObj.colonies[pObj.C[-1]] = deepcopy(pObj.colonies["pi_minus"])
        # assign the copied Pcolony to the cloned robot
        logging.debug("Robot %i got Pcolony %s" % (i, pObj.C[-1]) )
        robots.append(Kilobot(i, pObj.colonies[pObj.C[-1]]))
    #end for clones

    # initialize the simResult dictionary
    pObj.simResult = {colony_name: -1 for colony_name in pObj.C}
while (True):
    print("\n")
    
    if (type(pObj) == sim.Pcolony):
        robot.raw_input_state = bridge.getState(robot.uid)
        robot.procInputModule()
    else:
        for robot in robots:
            robot.raw_input_state = bridge.getState(robot.uid)
            robot.procInputModule()

    sim_result = pObj.runSimulationStep()
    # if the simmulation result is other than step finished (i.e. no_more_exec or error)
    if (sim_result != sim.SimStepResult.finished):
        # exit the loop
        logging.warn("Exiting loop")
        break

    if (type(pObj) == sim.Pcolony):
        robot.procOutputModule()
        bridge.setState(robot.uid, robot.output_state["motion"], robot.output_state["rgb_led"])
    else:
        for robot in robots:
            robot.procOutputModule()
            bridge.setState(robot.uid, robot.output_state["motion"], robot.output_state["rgb_led"])
