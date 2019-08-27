import time

from Robot.faultFreeRobot import FaultFreeRobot
from Robot.stationaryFaultRobot import StationaryFaultyRobot
from TverbergPoint.TverbergPoint import *

random.seed(1)

n = 100  # number of robots
n_faulty = 50  # number of faulty robots
n_fault_free = n - n_faulty  # number of fault-free robots

method = "center"  # "tver" for Tverberg point or "center" for centerpoint

p = random_point_set(n, lower=-10, upper=10)  # robot coordinates
p_fault_free = p[:n_fault_free]  # fault-free robot coordinates
p_faulty = p[n_fault_free:]  # faulty robot coordinates

delta = 1e-1  # halt parameter
sensDist = 10  # sensing range
alpha = 0.5  # velocity parameter
velMax = 2  # max velocity

rob_faulty = []
rob_fault_free = []

start_time = time.time()

for i in range(0, n_faulty):
    rob_faulty.append(StationaryFaultyRobot(p_faulty[i]))

for i in range(0, n_fault_free):
    rob_fault_free.append(FaultFreeRobot(p_fault_free[i], sensDist, alpha, velMax, delta))

while True:
    stop = True
    for i in range(0, n_fault_free):
        rob_fault_free[i].updatePos(p, method=method)
    for i in range(0, n_fault_free):
        p[i] = rob_fault_free[i].getPos()
        p_fault_free = p[:n_fault_free]
        p_faulty = p[n_fault_free:]
        stop = stop and rob_fault_free[i].stop

    plt.clf()
    prepare_axis(-10, 10, -10, 10)
    plot_point_set(p_fault_free, color='b')  # fault-free robots are plotted in blue
    plot_point_set(p_faulty, color='r')  # faulty robots are plotted in red
    plt.pause(1)

    if stop:
        break
print("Total time used: %.2f s" % (time.time() - start_time))
end = input('Press enter to end the program.')
