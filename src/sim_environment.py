# Acts as the environment for the RL problem.
# Uses TraCI to monitor/control SUMO simulations. Takes the given action provided and outputs the Next state and Reward.

# Depends on: plot_metrics.py

# Handled by Anirudh


from sumolib import checkBinary
import traci
import plot_metrics
import array
import os

T_SWITCH = 12                                               # Intersection switching time
T_TRANS = 4                                                 # Signal transition time when changing sides
JAMLEN_THRESH = 65                                          # Length of vehicle queue for jamming
NUM_INTN_SIDES = 15                                         # No. of intersection sides
NUM_LANES = [3, 2, 3, 2, 3, 2, 3, 2, 1, 1, 1, 1, 2, 1, 2]   # No. of lanes at intersection sides

TL_IDS = ["gneJ20", "gneJ18", "151147259", "gneJ69"]        # Intersection traffic light IDs

# Intersection phases
TL_PHASES = {'N1_G': 0, 'N1_Y': 1, 'N2_G': 2, 'N2_Y': 3, 'N3_G': 4, 'N3_Y': 5, 'N4_G': 6, 'N4_Y': 7,
             'N5_G': 0, 'N5_Y': 1, 'N6_G': 2, 'N6_Y': 3, 'N7_G': 4, 'N7_Y': 5, 'N8_G': 6, 'N8_Y': 7,
             'N9_G': 0, 'N9_Y': 1, 'N10_G': 2, 'N10_Y': 3, 'N11_G': 4, 'N11_Y': 5, 'N12_G': 6, 'N12_Y': 7,
             'N13_G': 0, 'N13_Y': 1, 'N14_G': 2, 'N14_Y': 3, 'N15_G': 4, 'N15_Y': 5}

SUMO_OP_FILE = "../scripts/txmap-tripinfo.xml"

sumoBinary = checkBinary('sumo-gui')                        # mode of SUMO

intn_prev_action = [1, 5, 9, 13]                            # previous action taken at the intersections

first_act_of_run = 0                                        # flag for occurrence of first action of a run

curr_state = array.array('i', [0] * NUM_INTN_SIDES)
prev_state = array.array('i', [0] * NUM_INTN_SIDES)
curr_jam_state = array.array('i', [0] * NUM_INTN_SIDES)
prev_jam_state = array.array('i', [0] * NUM_INTN_SIDES)


# Desc: Starts a new simulation run by generating a random routes file. Also reinitializes the detector counts etc.
# Inputs - run: run number, 0 implies first run
# Outputs - None
def start_new_run(run):

    global curr_state, prev_state, curr_jam_state, prev_jam_state, intn_prev_action, first_act_of_run

    curr_state = array.array('i', [0] * NUM_INTN_SIDES)
    prev_state = array.array('i', [0] * NUM_INTN_SIDES)
    curr_jam_state = array.array('i', [0] * NUM_INTN_SIDES)
    prev_jam_state = array.array('i', [0] * NUM_INTN_SIDES)
    intn_prev_action = [1, 5, 9, 13]
    first_act_of_run = 0

    # Setup plot_metrics for a new set of runs
    if run == 0:
        plot_metrics.init()

    # Generate a new random routes file
    os.system("python \"%SUMO_HOME%/tools/randomTrips.py\" -n ../scripts/txmap.net.xml --trip-attributes=\"type=\\\"light_norm_heavy\\\"\" "
              "-a ../scripts/txmap.add.xml -r ../scripts/txmap.rou.xml -e 2000 -p 0.5 --binomial=5")

    # Delete unwanted alt route file
    os.system("del \"../scripts/txmap.rou.alt.xml\"")

    # Delete unwanted trips file
    os.system("del trips.trips.xml")

    # Start SUMO and connect traCI to it
    traci.start([sumoBinary, "-c", "../scripts/txmap.sumocfg", "--gui-settings-file", "../scripts/guisettings.xml", "--start", "--quit-on-end"])

    return


# Desc: Gives Green signal to the given intersection side.
# Inputs - a: The intersection side to be given Green signal to, 1<=a<=15. If a = 0, no action is taken
# Outputs - A dictionary containing 2 elements
#               'rwd': Reward; Returns -100 if the current simulation is over
#               'next_state': 15-element array containing the no. of vehicles waiting at the intersections in new state
def take_action(a):

    global curr_state, prev_state, prev_jam_state, intn_prev_action, first_act_of_run

    # randomization phase, no control; let static TL logic of SUMO run
    if a == 0:
        for i in range(T_TRANS + T_SWITCH):
            traci.simulationStep()
        curr_state = get_current_state()
        R = calculate_reward(curr_jam_state, prev_jam_state)

        # all vehicles left simulation; current run over
        if traci.simulation.getMinExpectedNumber() == 0:
            traci.close()
            plot_metrics.record_metrics_of_run()
            return {'rwd': -100, 'next_state': curr_state}

        prev_state = curr_state
        prev_jam_state = curr_jam_state
        return {'rwd': R, 'next_state': curr_state}
    # randomization phase over, mark it in tripfile file
    elif first_act_of_run == 0:
        with open(SUMO_OP_FILE, 'a') as tripfile:
            print('\t' + "$FIRSTACTION", file=tripfile)
        first_act_of_run = 1

    # Intersection being controlled
    if 1 <= a <= 4:
        intn = 0
    elif 5 <= a <= 8:
        intn = 1
    elif 9 <= a <= 12:
        intn = 2
    else:
        intn = 3

    # if action same as previous for this intersection
    if a == intn_prev_action[intn]:
        traci.trafficlight.setPhase(TL_IDS[intn], TL_PHASES['N' + str(a) + '_G'])   # Continue Green during transition
        for i in range(T_TRANS):
            traci.simulationStep()
    # different action
    else:
        traci.trafficlight.setPhase(TL_IDS[intn], TL_PHASES['N' + str(intn_prev_action[intn]) + '_Y'])   # Give Yellow during transition
        for i in range(T_TRANS):
            traci.simulationStep()

    # Give Green and wait for T_SWITCH secs
    traci.trafficlight.setPhase(TL_IDS[intn], TL_PHASES['N' + str(a) + '_G'])
    for i in range(T_SWITCH):
        traci.simulationStep()

    curr_state = get_current_state()

    with open(SUMO_OP_FILE, 'a') as tripfile:
        print('\t' + "<qlength vals=\"" + str(curr_state.tolist()) + "\"/>", file=tripfile)

    # all vehicles left simulation; current run over
    if traci.simulation.getMinExpectedNumber() == 0:
        traci.close()
        plot_metrics.record_metrics_of_run()
        return {'rwd': -100, 'next_state': curr_state}

    R = calculate_reward(curr_jam_state, prev_jam_state)

    prev_state = curr_state
    prev_jam_state = curr_jam_state
    intn_prev_action[intn] = a

    return {'rwd': R, 'next_state': curr_state}


# Desc: Returns the no. of vehicles waiting at the intersections currently.
# Inputs - None
# Outputs - state: 15-element array containing the no. of vehicles waiting at the intersections currently
def get_current_state():

    global curr_jam_state

    state = array.array('i', [0] * NUM_INTN_SIDES)

    for i in range(NUM_INTN_SIDES):
        n = 0
        jammed = 0
        for j in range(NUM_LANES[i]):
            n = max(n, traci.lanearea.getJamLengthVehicle("e2det_N"+str(i+1)+"_"+str(j)))
            if traci.lanearea.getJamLengthMeters("e2det_N"+str(i+1)+"_"+str(j)) > JAMLEN_THRESH:
                jammed = 1

        if jammed == 1:
            state[i] = int(n*1.5 + 1)      # if jammed, over-estimate possible queue length
        else:
            state[i] = n

        curr_jam_state[i] = jammed

    return state


# Desc: Calculates the reward when transitioning states.
# Inputs - curr_jam: Current state of congestion
#          prev_jam: Previous state of congestion
# Outputs - Reward: Reward
def calculate_reward(curr_jam, prev_jam):

    Reward = 0
    for i in range(NUM_INTN_SIDES):
        if curr_jam[i]==0 and prev_jam[i]==0:
            Reward += 0
        if curr_jam[i]==1 and prev_jam[i]==0:
            Reward += -1
        if curr_jam[i]==0 and prev_jam[i]==1:
            Reward += 1
        if curr_jam[i]==1 and prev_jam[i]==1:
            Reward += -2

    Reward /= NUM_INTN_SIDES

    return Reward


# Desc: Enable/Disable GUI mode of SUMO
# Inputs - mode: 1 = GUI mode, 0 = non-GUI mode
# Outputs - None
def endis_sumo_guimode(mode):

    global sumoBinary

    if mode == 1:
        sumoBinary = checkBinary('sumo-gui')
    else:
        sumoBinary = checkBinary('sumo')

    return
