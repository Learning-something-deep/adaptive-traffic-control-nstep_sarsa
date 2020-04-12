# Main module; compares the performance of n-step Sarsa and LQF algorithms by plotting various metrics
# Evaluates the effect of changing the parameters of n-step Sarsa to find optimal values

# Depends on: sarsa_nstep.py, lqf_algo.py, plot_metrics.py

# Handled by All


import sim_environment
import sarsa_nstep


# Estimate q* using n-step SARSA
sim_environment.endis_sumo_guimode(0)   # SUMO non-GUI mode
n = 4; c = 0.1; epsilon = 1.0; Nruns = 5
W = sarsa_nstep.sarsa_nstep_diff_train(n, c, epsilon, Nruns)
print(W)

# Try out performance with the estimated weights
sim_environment.endis_sumo_guimode(1)   # SUMO GUI mode
Nuns = 5
sarsa_nstep.sarsa_nstep_diff_live(W, Nuns)
