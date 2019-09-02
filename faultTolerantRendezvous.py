import time

from Robot.faultFreeRobot import FaultFreeRobot
from Robot.stationaryFaultRobot import StationaryFaultyRobot
from TverbergPoint.TverbergPoint import *

random.seed(1)

n = 100  # number of robots
n_faulty = 33  # number of faulty robots
n_fault_free = n - n_faulty  # number of fault-free robots

box = 1  # box size

method = "center"  # "tver" for Tverberg point or "center" for centerpoint

p = random_point_set(n, lower=-box, upper=box)  # robot coordinates
p_fault_free = p[:n_fault_free]  # fault-free robot coordinates
p_faulty = p[n_fault_free:]  # faulty robot coordinates

delta = 0.01 * box  # halt parameter
sensDist = 3  # sensing range
alpha = 0.5  # velocity parameter
velMax = 0.2 * box  # max velocity

rob_faulty = []
rob_fault_free = []

start_time = time.time()

for i in range(0, n_faulty):
    rob_faulty.append(StationaryFaultyRobot(p_faulty[i]))
    p[n_fault_free + i] = p_faulty[i]

for i in range(0, n_fault_free):
    rob_fault_free.append(FaultFreeRobot(p_fault_free[i], sensDist, alpha, velMax, delta))

plt.figure(figsize=(2, 2))
t = 0

while True:
    stop = True

    plt.clf()
    plt.grid(True, which='major')
    ax = plt.gca()
    ax.set_xlim(-box, box)
    ax.set_ylim(-box, box)
    # plt.xticks([0.3*i for i in range(-5,5, 1)])
    # plt.yticks([0.3*i for i in range(-5,5, 1)])
    plt.gca().set_aspect('equal', adjustable='box')
    # ax.grid(True)
    for tic in ax.xaxis.get_major_ticks():
        tic.tick1On = tic.tick2On = False
        # tic.label1On = tic.label2On = False
    for tic in ax.yaxis.get_major_ticks():
        tic.tick1On = tic.tick2On = False
        # tic.label1On = tic.label2On = False
    start, end = ax.get_xlim()
    ax.xaxis.set_ticks(np.arange(start, end + 0.01, 0.2))
    ax.yaxis.set_ticks(np.arange(start, end + 0.01, 0.2))

    # ax.set_xticks([0, 0.3, 0.4, 1.0, 1.5])
    ax.set_xticklabels([-1, "", "", "", "", 0, "", "", "", "", 1])
    ax.set_yticklabels([-1, "", "", "", "", 0, "", "", "", "", 1])

    plot_point_set(p_fault_free, color='b')  # fault-free robots are plotted in blue
    plot_point_set(p_faulty, color='r')  # faulty robots are plotted in red
    plt.pause(1)
    plt.savefig('./figure/faultTolerantRendezvous/%s%d.eps' % (method, t))
    # end = input('Press enter to end the program.')
    t += 1

    for i in range(0, n_fault_free):
        rob_fault_free[i].updatePos(p, method=method)
    for i in range(0, n_fault_free):
        p[i] = rob_fault_free[i].getPos()
        stop = stop and rob_fault_free[i].stop
    p_fault_free = p[:n_fault_free]
    p_faulty = p[n_fault_free:]

    if stop:
        break
print("Total time used: %.2f s" % (time.time() - start_time))
end = input('Press enter to end the program.')
