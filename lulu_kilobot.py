import logging
import colorlog # colors log output
from vrep_bridge import vrep_bridge # for getState, setState
from lulu_pcol_sim import sim

def procInputModule(raw_state, colony):
    """Process raw_state info received from sensors and populate the input module agents with significant objects

    :raw_state: structured dictionary obtained from vrep_bridge.getState()
    :colony: Pcolony
    """
    pass
# end procInputModule()

def procOutputModule(colony):
    """Process the objects present in the output module agents and transform them into commands that can be sent to the kilobot through setState()

    :colony: Pcolony
    :returns: motion (vrep_bridge.Motion) motion type
    :returns: light [r, g, b] with values between 0-2

    """
    motion = vrep_bridge.Motion.stop # default motion
    light = [0, 0, 0] # default color (off)

    if ('m_S' in colony.agents['AG_motion'].obj):
        motion = vrep_bridge.Motion.forward
    elif ('m_L' in colony.agents['AG_motion'].obj):
        motion = vrep_bridge.Motion.left
    elif ('m_R' in colony.agents['AG_motion'].obj):
        motion = vrep_bridge.Motion.right
    
    return motion, light
#end procOutputModule

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
colorlog.basicConfig(level = logging.INFO)
stream = colorlog.root.handlers[0]
stream.setFormatter(formatter);

# read Pcolony from file
colony = sim.readInputFile("input.txt")
# make link with v-rep
bridge = vrep_bridge.VrepBridge()

while (True):
    print("\n")
    raw_state = bridge.getState()
    procInputModule(raw_state, colony)
    
    sim_result = colony.runSimulationStep()
    # if the simmulation result is other than step finished (i.e. no_more_exec or error)
    if (sim_result != sim.SimStepResult.finished):
        # exit the loop
        logging.warn("Exiting loop")
        break

    motion, light = procOutputModule(colony)
    bridge.setState(motion, light)
