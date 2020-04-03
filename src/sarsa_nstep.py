"""
Implements Differential semi-gradient n-step Sarsa for estimating q = q*
Implemented according to pseudo-code given in Sec 10.5 of Sutton & Bartos's RL book
Takes as input No. of runs, n, alpha, beta, epsilon. Outputs the final weights after all the runs.

Depends on: sim_environment.py
"""
import numpy as np
import random
import math

import sim_environment


def state_dependency(s1, s2):
    return math.sqrt(s1 * s1 + s2 * s2)


def phi(s, a):
    s_sqr = list(map(lambda x: x * x, s))
    s_len = len(s)
    a_len = s_len
    arr = np.zeros([s_len * a_len + 1, 1])
    for i in range(s_len):
        arr[i + a_len * (a - 1)] = s_sqr[i]
    arr[-1] = 1
    return arr


def initial_state_generate():
    for j in range(np.random.choice([1, 4, 8, 12, 16, 20])):
        env_dict = sim_environment.take_action(0)
    return env_dict['next_state']


# Desc: Computes the approximate action-value function q(s, a) using Linear function approximation.
# Inputs - s: 15-element array containing the no. of vehicles waiting at the intersections in state s
#          a: Action taken in state s, 1<=a<=15
#          w: 556-element weight vector, with the last element corresponding to bias
# Outputs - q_val: Calculated q(s, a) value

def q_est(s, a, w):
    q_val = np.dot(w, phi(s, a))
    return q_val


def epsilon_greedy_a(e, a_space, next_s, w):
    random_e = random.uniform(0, 1)
    if random_e < e:
        return np.random.choice(a_space)
    else:
        q_next = []
        for a_temp in a_space:
            q_next.append(q_est(next_s, a_temp, w))
        q_max = max(q_next)
        q_max_index = [i for i, j in enumerate(q_next) if j == q_max]
        rand_greedy_q = np.random.choice(q_max_index)
        return a_space[rand_greedy_q]


# Desc: Runs the Differential n-step Sarsa algorithm for estimating optimal action-value function using Linear
#       function approximation.
# Inputs - n: n-step bootstrapping
#          alpha: step size for weight updates
#          beta: step size for avg reward updates
#          epsilon: exploration parameter for epsilon-greedy
#          Nruns: No. of runs
# Outputs - W: Trained weights after all runs


def sarsa_nstep_diff_train(n, c, epsilon, Nruns):
    # Note: functions used by this function
    # sim_environment.start_new_run(run)
    # sim_environment.take_action(a)
    # sim_environment.get_current_state()
    # q_est(s, a, w)

    print("Running nstep SARSA training")
    run_len = 5000
    s_len = 15
    a_len = 15
    buff_len = n + 1
    r_arr = np.zeros(buff_len, dtype=int)
    a_arr = np.zeros(buff_len, dtype=int)
    s_arr = []
    weight = np.zeros([s_len * a_len + 1, 1])
    avg_reward = 0
    for run in range(Nruns):
        print("Run i =" + str(run))
        e = epsilon * (1 / math.ceil((run + 1) / 3))
        sim_environment.start_new_run(run)
        curr_s = initial_state_generate()
        curr_a = random.randint(1, 4)
        a_arr[0] = curr_a
        s_arr.insert(0, curr_s)
        for t in range(run_len):
            next_intersection = (t + 1) % 4
            if next_intersection == 3:
                a_space = [13, 14, 15]
            else:
                a_space = [4 * next_intersection + 1, 4 * next_intersection + 2, 4 * next_intersection + 3,
                           4 * next_intersection + 4]
            alpha = 1 / (math.ceil((t + 1) / 10))
            beta = c * alpha
            env_param = sim_environment.take_action(curr_a)
            r = env_param['rwd']
            next_s = env_param['next_state']
            r_arr[(t+1) % (n+1)] = r
            next_a = epsilon_greedy_a(e, a_space, next_s, weight[:, 0])
            s_arr.insert((t+1) % (n+1), next_s)
            a_arr[(t+1)%(n+1)] = next_a
            tau = t - n + 1
            if tau >= 0:
                q_tau_n = q_est(s_arr[(tau + n) % (n + 1)], a_arr[(tau + n) % (n + 1)], weight[:, 0])
                q_tau = q_est(s_arr[(tau) % (n + 1)], a_arr[(tau) % (n + 1)], weight[:, 0])
                do_error = sum(r_arr) - n * avg_reward + q_tau_n - q_tau
                avg_reward = avg_reward + beta * do_error
                weight[:, 0] = weight[:, 0] + alpha * do_error * np.transpose(phi(s_arr[tau % (n + 1)], a_arr[tau % (n + 1)]))
    W =  weight[:, 0]
    return W


# Desc: Uses the previously trained weights by n-step Sarsa to pick optimal actions.
# Inputs - W: Trained weights using n-step Sarsa
#          Nruns: No. of runs
# Outputs - None
def sarsa_nstep_diff_live(W, Nruns):

    # Note: functions used by this function
    # sim_environment.start_new_run(run)
    # sim_environment.take_action(a)
    # q_est(s, a, w)

    return
