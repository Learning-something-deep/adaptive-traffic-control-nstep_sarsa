# Parses the output of SUMO simulation, and plots congestion-related performance metrics.
# Plots Average queue length at intersections, Average arrival time, Average waiting time, and Average time loss (averaged 
# across simulation runs). Uses the output dump file of SUMO for each run as input.

# Depends on: None

# Handled by Anirudh


from lxml import etree as ET
import matplotlib.pyplot as plt

TRIPINFO_FILE = "../scripts/txmap-tripinfo.xml"
QLENGTH_FILE = "../scripts/qlengths.xml"

STEP_SIZE = 0
run = 0

# 2D arrays to store metrics of all runs
timeLoss_all_runs = []
waitingTime_all_runs = []
qlength_all_runs = []


# Desc: Clears any existing data for all runs, and starts fresh.
# Inputs - step_size: The number of simulation steps that equal one time step
# Outputs - None
def init(step_size):

    global STEP_SIZE, run, timeLoss_all_runs, waitingTime_all_runs, qlength_all_runs

    STEP_SIZE = step_size
    run = 0

    timeLoss_all_runs = []
    waitingTime_all_runs = []
    qlength_all_runs = []

    return


# Desc: Parses the simulation output file at the end of run. Performs averaging, and stores various vehicle metrics
#       for the run.
# Inputs - skip_time: simulation steps spent in initial randomization phase
# Outputs - None
def record_metrics_of_run(skip_time):

    global run, timeLoss_all_runs, waitingTime_all_runs

    # parse output and put the lines in an array
    elementList = []
    tree = ET.parse(TRIPINFO_FILE)
    root = tree.getroot()
    nlines = 0
    for element in root.iter('tripinfo'):
        elementList.append(element)
        nlines += 1

    # calculate the average metrics for each time step
    i = 0
    tstep = 0
    lineno = 0
    t = skip_time
    timeLoss_arr = []
    waitingTime_arr = []
    timeLoss_arr_temp = []
    waitingTime_arr_temp = []
    while lineno < nlines:

        element = elementList[lineno]
        arrival = float(element.get('arrival'))

        if arrival < skip_time:             # discard line
            lineno += 1
            continue
        elif arrival <= t + STEP_SIZE:      # read metrics from line
            duration = float(element.get('duration'))
            timeLoss_arr_temp.insert(i, float(element.get('timeLoss'))/duration)
            waitingTime_arr_temp.insert(i, float(element.get('waitingTime'))/duration)
            i += 1
            lineno += 1
        else:                               # average metrics and move on to next time step
            if i == 0:              # no data points for this time step, fill it with previous time step's
                if tstep == 0:
                    timeLoss_arr.insert(tstep, 0.0)
                    waitingTime_arr.insert(tstep, 0.0)
                else:
                    timeLoss_arr.insert(tstep, timeLoss_arr[tstep - 1])
                    waitingTime_arr.insert(tstep, waitingTime_arr[tstep - 1])
            else:
                timeLoss_arr.insert(tstep, average(timeLoss_arr_temp))
                waitingTime_arr.insert(tstep, average(waitingTime_arr_temp))

            t += STEP_SIZE
            tstep += 1

            i = 0
            timeLoss_arr_temp = []
            waitingTime_arr_temp = []
    # add the last element as well
    timeLoss_arr.insert(tstep, average(timeLoss_arr_temp))
    waitingTime_arr.insert(tstep, average(waitingTime_arr_temp))

    # record queue length values
    qlength_arr = []
    tree = ET.parse(QLENGTH_FILE)
    root = tree.getroot()
    tstep = 0
    for element in root.iter('qlength'):
        vals = (element.get('vals'))[1:-1]
        qlens = [float(n) for n in vals.split(", ")]
        qlength_arr.insert(tstep, average(qlens))
        tstep += 1

    timeLoss_all_runs.insert(run, timeLoss_arr)
    waitingTime_all_runs.insert(run, waitingTime_arr)
    qlength_all_runs.insert(run, qlength_arr)
    run += 1

    return


# Desc: Plots all the metrics, averaged across runs, vs time.
# Inputs - title: title of graph
# Outputs - None
def plot_all_metrics(title):

    fig1 = plt.figure()
    fig1.suptitle(title)
    plt.plot(average_metrics(timeLoss_all_runs), 'b')
    plt.xlabel('time steps')
    plt.ylabel('timeLoss')

    fig2 = plt.figure()
    fig2.suptitle(title)
    plt.plot(average_metrics(waitingTime_all_runs), 'r')
    plt.xlabel('time steps')
    plt.ylabel('waitingTime')

    fig3 = plt.figure()
    fig3.suptitle(title)
    plt.plot(average_metrics(qlength_all_runs), 'g')
    plt.xlabel('time steps')
    plt.ylabel('Queue length')

    plt.show(block=False)

    return


# Average given metric across metrics
def average_metrics(metric_all_runs):

    minlen = len(metric_all_runs[0])
    for i in range(run):
        minlen = min(minlen, len(metric_all_runs[i]))

    avg_metric = [0.0] * minlen
    for i in range(run):
        avg_metric = [avg_metric[j] + metric_all_runs[i][j] for j in range(minlen)]

    avg_metric = [avg_metric[j]/run for j in range(minlen)]

    return avg_metric


# Calculates average of a list
def average(lst):
    return sum(lst) / len(lst)
