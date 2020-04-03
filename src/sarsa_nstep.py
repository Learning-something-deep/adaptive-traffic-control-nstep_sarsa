# Implements Differential semi-gradient n-step Sarsa for estimating q = q*
# Implemented according to pseudo-code given in Sec 10.5 of Sutton & Bartos's RL book
# Takes as input No. of runs, n, alpha, beta, epsilon. Outputs the final weights after all the runs.

# Depends on: sim_environment.py

# Handled by Akshay


# Desc: Runs the Differential n-step Sarsa algorithm for estimating optimal action-value function using Linear
#       function approximation.
# Inputs - n: n-step bootstrapping
#          c: Step-size ratio; beta = c*alpha 
#          epsilon: Initial exploration parameter for epsilon-greedy
#          Nruns: No. of runs
# Outputs - W: Trained weights after all runs
def sarsa_nstep_diff(n, c, epsilon, Nruns):

    # Note: functions used by this function
    # sim_environment.start_new_run(run)
    # sim_environment.take_action(a)
    # q_est(s, a, w)

    return W



# Desc: Computes the approximate action-value function q(s, a) using Linear function approximation.
# Inputs - s: 15-element array containing the no. of vehicles waiting at the intersections in state s
#          a: Action taken in state s, 1<=a<=15
#          w: 556-element weight vector, with the last element corresponding to bias
# Outputs - q_val: Calculated q(s, a) value
def q_est(s, a, w):



    return q_val

