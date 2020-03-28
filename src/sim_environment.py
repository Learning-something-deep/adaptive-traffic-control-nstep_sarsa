# Acts as the environment for the RL problem.
# Uses TraCI to monitor/control SUMO simulations. Takes the given action provided and outputs the Next state and Reward.

# Depends on: plot_metrics.py

# Handled by Anirudh


# Desc: Starts a new simulation run by generating a random routes file. Also reinitializes the detector counts etc.
# Inputs - run: run number, 0 implies first run
# Outputs - None
def start_new_run(run):

    # Note: functions used by this function
    # plot_metrics.init() for the first run
    # plot_metrics.record_metrics_of_run()

    return



# Desc: Gives Green signal to the given intersection side.
# Inputs - a: The intersection side to be given Green signal to, 1<=a<=15
# Outputs - A dictionary containing 2 elements
#               'rwd': Reward; Returns -100 if the current simulation is over
#               'next_state': 15-element array containing the no. of vehicles waiting at the intersections in new state
def take_action(a):


    return {'rwd': R, 'next_state': Next_state}



# Desc: Returns the no. of vehicles waiting at the intersections in the current state.
# Inputs - None
# Outputs - 15-element array containing the no. of vehicles waiting at the intersections currently
def get_current_state():


    return current_state

