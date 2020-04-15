# Main module; compares the performance of n-step Sarsa and LQF algorithms by plotting various metrics
# Evaluates the effect of changing the parameters of n-step Sarsa to find optimal values

# Depends on: sarsa_nstep.py, lqf_algo.py, static_signalling.py, plot_metrics.py

# Handled by All


import sim_environment
import static_signalling
import lqf_algo
import sarsa_nstep
import plot_metrics
from matplotlib import pyplot as plt


# Static signalling
sim_environment.endis_sumo_guimode(1)
Nruns = 1
static_signalling.static_signalling(Nruns)
plot_metrics.plot_all_metrics("Static signalling")


# Longest Queue First (LQF) algorithm
sim_environment.endis_sumo_guimode(1)
Nruns = 1
lqf_algo.lqf(Nruns)
plot_metrics.plot_all_metrics("LQF algo")


# Estimate q* using n-step SARSA
sim_environment.endis_sumo_guimode(1)
n = 4; c = 0.1; epsilon = 1.0; Nruns = 5
W = sarsa_nstep.sarsa_nstep_diff_train(n, c, epsilon, Nruns)

# Try out performance with the estimated weights
sim_environment.endis_sumo_guimode(1)
Nuns = 3
sarsa_nstep.sarsa_nstep_diff_live(W, Nuns)
plot_metrics.plot_all_metrics("Sarsa algo")


plt.show()      # to prevent figures from closing
