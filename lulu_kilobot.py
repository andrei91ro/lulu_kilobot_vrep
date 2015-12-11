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

class Config():

    """Class used to store a parsed config file. This object is used to determine the behaviour of the application at runtime"""

    def __init__(self):
        self.C = [] # list of colony names
        self.nrRobots = 0 # nr of simulated robots
        self.nrRobotsPerColony = {} # dictionary (colony_name : nr_robots_that_will_use_this_colony}
        self.robotColony = [] # list [robot_nr] = "name_of_colony_that_will_be_used"
        self.robotName = [] # list of robot names (generated from their uid) ex ['robot_0', 'robot_1']
        self.nrAsignedRobotsPerColony = {} # dictionary (colony_name : nr_robots_that_have_been_asigned_this_colony_so_far)
# end class Config

def process_config_tokens(tokens, parent, index):
    """Process tokens recurently and return a Config object (or a subcomponent of the same type as parent)

    :tokens: the list of tokens to be processed
    :parent: an object that represents the type of the result
    :index: the start index in the list of tokens
    :returns: index - the current index in the token list (after finishing this component)
    :returns: result - an object that is the result of processing the input parent and tokens
    
    """
    logging.debug("process_tokens (parent_type = %s, index = %d)" % (type(parent), index))
    result = parent # construct the result of specified type
    prev_token = tokens[index]
    
    while (index < len(tokens)):
        token = tokens[index]
        logging.debug("token = '%s'" % token.value)
        
        if (type(parent) == Config):
            if (token.type == 'ASSIGN'):
                if (prev_token.value == "C"):
                    logging.info("building list")
                    index, result.C = process_config_tokens(tokens, result.C, index + 1)
                    # no robots have been asigned to any colony
                    result.nrAsignedRobotsPerColony = {colonyName: 0 for colonyName in result.C}
                
                elif (prev_token.value == 'nrRobots'):
                    logging.info("setting value");
                    index, result.nrRobots = process_config_tokens(tokens, result.nrRobots, index + 1)
                    # generate a list of robot names that have the uid appended at the end
                    result.robotName = ["robot_%d" % i for i in range(result.nrRobots)]
                    # generate a list of empty colony name assignments for each robot number
                    result.robotColony = [""] * result.nrRobots

                # if the previous token is a robot name
                elif (prev_token.value in result.robotName):
                    logging.info("processing Robot_name = Colony_name")
                    # obtain the robot number (uid to be more precise) from the robot name
                    # ex 2 from robot_2
                    robotNumber = result.robotName.index(prev_token.value)
                    logging.debug("robot_name = %s, number = %d" % (prev_token.value, robotNumber))
                    # we obtain the colony name (right hand side of the atribution)
                    index, colonyName = process_config_tokens(tokens, "", index + 1)
                    # if this is a known colony name
                    if (colonyName in result.C):
                        logging.debug("%s = %s" %(prev_token.value, colonyName))
                        # asign the read colony name to the robot
                        # so at the init phase the correct Pcolony object will be assigned to this robot
                        result.robotColony[robotNumber] = colonyName
                        result.nrAsignedRobotsPerColony[colonyName] += 1

                # if the previous token is a colony name
                elif (prev_token.value in result.C):
                    logging.info("processing Colony_name = number_of_robots_on_colony")
                    index, numberOfRobotsOnColony = process_config_tokens(tokens, int(), index + 1)
                    # numberOfRobotsOnColony will be assigned the colony specified on the left hand side of the atribution
                    result.nrRobotsPerColony[prev_token.value] = numberOfRobotsOnColony

        elif (type(parent) == list):
            logging.debug("processing as List")
            if (token.type == 'ID'):
                result.append(token.value);
        
        elif (type(parent) == str):
            logging.debug("processing as Str")
            if (token.type == 'ID'):
                result = token.value;

        elif (type(parent) == int):
            logging.debug("processing as Int")
            if (token.type == 'NUMBER'):
                result = int(token.value);
        
        if (token.type == 'END'):
            logging.info("finished this block with result = %s" % result)
            return index, result;
        
        prev_token = token;
        index += 1
    # end while

    return index, result
#end process_config_tokens()

def readConfigFile(filename, printTokens=False):
    """Parses the given config file and produces a Config object

    :filename: string path to the file that will be parsed
    :returns: Config object

    """
    logging.info("Reading input file")

    with open(filename) as file_in:
        lines = "".join(file_in.readlines());

    # construct array of tokens for later use
    tokens = [token for token in sim.tokenize(lines)];

    index, config = process_config_tokens(tokens, Config(), 0)
    
    nrUnasignedRobots = config.robotColony.count("")
    nrUnasignedColonies = sum(v for k, v in config.nrRobotsPerColony.items())
    logging.debug("Nr robots without asigned colonies = %d" % nrUnasignedRobots)
    logging.debug("Nr colonies without asigned robots = %d" % nrUnasignedColonies)
    
    if (nrUnasignedRobots > 0):
        if (nrUnasignedRobots > nrUnasignedColonies):
            logging.error("There are not enough unasigned colonies in order to ensure that each of the %d robots has a Pcolony asociated with it" % config.nrRobots)
        else:
            while (nrUnasignedRobots > 0):
                unasignedRobotIndex = config.robotColony.index("")
                for colonyName in config.C:
                    # if the currently assigned number of robots is less than what the user requested
                    if (config.nrAsignedRobotsPerColony[colonyName] < config.nrRobotsPerColony[colonyName]):
                        # assign colonyName with the first unasigned robot
                        config.robotColony[unasignedRobotIndex] = colonyName
                        # increase the number of assignments for this colony
                        config.nrAsignedRobotsPerColony[colonyName] += 1
                        # decrease the number of unasigned robots
                        nrUnasignedRobots -= 1
                        logging.debug("unasignedRobotIndex = %d, robotColony = %s, nrRobotsPerColony = %s, nrAsignedRobotsPerColony = %s" % (unasignedRobotIndex, config.robotColony, config.nrRobotsPerColony, config.nrAsignedRobotsPerColony))
                        break

    return config
# end readConfigFile()

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
# if the p object read from the input file is a Pswarm
if (type(pObj) == sim.Pswarm):
    if (len(sys.argv) < 3):
        logging.error("Expected config file path as second parameter")
        exit(1)

    config = readConfigFile(sys.argv[2])
# make link with v-rep
bridge = vrep_bridge.VrepBridge()

if (type(pObj) == sim.Pcolony):
    robot = Kilobot(0, pObj)
else:
    # array of Kilobot objects
    robots = [] 
    # used to determine how many robots have been set up up so far with this colony name
    # so that the first one gets the original colony and the others get a clone
    config.nrConfiguredRobotsWithColony = {colonyName: 0 for colonyName in config.nrAsignedRobotsPerColony.keys()}

    # spawn n-1 robots because 1 is already in the scene and is copied
    bridge.spawnRobots(nr = config.nrRobots - 1)

    # create aditional copies of the Pcolony and assign then to each robot
    for i in range(config.nrRobots):
        # if i am the first robot that uses this Pcolony
        if (config.nrConfiguredRobotsWithColony[config.robotColony[i]] == 0): 
            logging.debug("Robot %d is the first to use %s Pcolony" % (i, config.robotColony[i]))
            robots.append(Kilobot(i, pObj.colonies[config.robotColony[i]]))
            # increase the nr of robots configured with this colony
            config.nrConfiguredRobotsWithColony[config.robotColony[i]] += 1
        else:
            logging.debug("Copying Pcolony %s for robot %d" % (config.robotColony[i], i))
            # add a new Pcolony name (with uid appended)
            pObj.C.append(config.robotColony[i] + "_" + str(i))
            logging.debug("pObj.C = %s" % pObj.C)
            # create a value copy of the colony and store it under the new name
            pObj.colonies[pObj.C[-1]] = deepcopy(pObj.colonies[config.robotColony[i]])
            # assign the copied Pcolony to the cloned robot
            logging.debug("Robot %i got Pcolony %s" % (i, pObj.C[-1]) )
            robots.append(Kilobot(i, pObj.colonies[pObj.C[-1]]))
            # increase the nr of robots configured with this colony
            config.nrConfiguredRobotsWithColony[config.robotColony[i]] += 1
            # change the generic colony name to the real allocated one
            config.robotColony[i] = pObj.C[-1]
    #end for clones

    print("\n Robot - Pcolony association table:")
    print("robot_id    colony_name\n")
    for i in range(config.nrRobots):
        print("robot_%d    %s" % (i, config.robotColony[i]))
    print("\n")

    # initialize the simResult dictionary
    pObj.simResult = {colonyName: -1 for colonyName in pObj.C}
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
# end while

# show remove clone confirmation only when simulating Pswarms
if (type(pObj) == sim.Pswarm):
    confirmRemoveRobots = input("Remove cloned robots from scene ? (y/n)")
    if (confirmRemoveRobots in ('y', 'Y')):
        bridge.removeRobots()
