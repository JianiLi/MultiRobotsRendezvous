from Robot.robot import *
from centerpoint.Centerpoint import *
from TverbergPoint.TverbergPoint import *
random.seed(1)

n = 100
n_faulty = 30
n_fault_free = n - n_faulty

p = random_point_set(n, lower=-10, upper=10)
p_fault_free = p[:n_fault_free]
p_faulty = p[n_fault_free:]

delta = 1e-3
sensDist = 5
alpha = 0.5
velMax = 1

rob_faulty = []
rob_fault_free = []

for i in range(0, n_faulty):
    rob_faulty.append(FaultyRobot(p_faulty[i]))

for i in range(0,n_fault_free):
    rob_fault_free.append(FaultFreeRobot(p_fault_free[i], sensDist, alpha, velMax))

while True:
    for i in range(0, n_fault_free):
        rob_fault_free[i].updatePos(p)
    for i in range(0,n_fault_free):
        p[i] = rob_fault_free[i].getPos()
        p_fault_free = p[:n_fault_free]
        p_faulty = p[n_fault_free:]

    plt.clf()
    x_min, x_max = find_x_bounds(p)
    interval = Interval(x_min, x_max)
    y_min, y_max = find_y_bounds(p)
    prepare_axis(-10, 10,-10,10)
    plot_point_set(p_fault_free, color='b')
    plot_point_set(p_faulty, color='r')
    plt.pause(1)
    #end = input('Press enter to the next step.')
