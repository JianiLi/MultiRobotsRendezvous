import time

from Robot.faultFreeRobot import FaultFreeRobot
from Robot.stationaryFaultRobot import StationaryFaultyRobot
from TverbergPoint.TverbergPoint import *

random.seed(1)

n = 26  # number of robots
n_faulty = 12  # number of faulty robots

n_fault_free = n - n_faulty  # number of fault-free robots

box = 1  # box size

method = "tver"  # "tver" for Tverberg point or "center" for centerpoint

p = random_point_set(n, lower=-box, upper=box)  # robot coordinates
p_fault_free = p[:n_fault_free]  # fault-free robot coordinates
p_faulty = p[n_fault_free:]  # faulty robot coordinates

delta = 1e-7 * box  # halt parameter
sensDist = 1.5  # sensing range
alpha = 0.5  # velocity parameter
velMax = 0.2 * box  # max velocity

rob_faulty = []
rob_fault_free = []

start_time = time.time()

rand_addon = 0.0

for i in range(0, int(n_faulty / 2)):
    p_faulty[i] = Point(-1.4 + rand_addon * random.random(), 0 + rand_addon * random.random())
    rob_faulty.append(StationaryFaultyRobot(p_faulty[i]))
    p[n_fault_free + i] = p_faulty[i]

for i in range(int(n_faulty / 2), n_faulty):
    p_faulty[i] = Point(1.4 + rand_addon * random.random(), 0 + rand_addon * random.random())
    rob_faulty.append(StationaryFaultyRobot(p_faulty[i]))
    p[n_fault_free + i] = p_faulty[i]

for i in range(0, int(n_fault_free / 2)):
    p_fault_free[i] = Point(-0.5 + rand_addon * random.random(), 0 + rand_addon * random.random())
    p[i] = p_fault_free[i]
    rob_fault_free.append(FaultFreeRobot(p_fault_free[i], sensDist, alpha, velMax, delta))

for i in range(int(n_fault_free / 2), n_fault_free):
    p_fault_free[i] = Point(0.5 + rand_addon * random.random(), 0 + rand_addon * random.random())
    p[i] = p_fault_free[i]
    rob_fault_free.append(FaultFreeRobot(p_fault_free[i], sensDist, alpha, velMax, delta))

plt.figure(figsize=(2, 2))

while True:
    plt.clf()
    plt.grid(True, which='major')
    ax = plt.gca()
    ax.set_xlim(-box - 0.5, box + 0.5)
    ax.set_ylim(-box - 0.5, box + 0.5)
    plt.xticks([])
    plt.yticks([])
    plt.gca().set_aspect('equal', adjustable='box')

    plot_point_set(p_fault_free, color='b')  # fault-free robots are plotted in blue
    plot_point_set(p_faulty, color='r')  # faulty robots are plotted in red
    plt.pause(1)

    stop = True
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
