import time

from matplotlib.ticker import MaxNLocator

from Robot.faultFreeRobot import FaultFreeRobot
from Robot.stationaryFaultRobot import StationaryFaultyRobot
from TverbergPoint.TverbergPoint import *

random.seed(1)

n = 40  # number of robots
n_faulty = 5  # number of faulty robots
n_fault_free = n - n_faulty  # number of fault-free robots

box = 1  # box size

fixNeighbor = True
method = "center"  # "tver" for Tverberg point or "center" for centerpoint

p = random_point_set(n, lower=-box, upper=box)  # robot coordinates
p_fault_free = p[:n_fault_free]  # fault-free robot coordinates
p_faulty = p[n_fault_free:]  # faulty robot coordinates

delta = 0.001 * box  # halt parameter
sensDist = 0.8  # sensing range
alpha = 0.8  # velocity parameter
velMax = 0.5 * box  # max velocity

rob_faulty = []
rob_fault_free = []

start_time = time.time()

for i in range(0, n_faulty):
    rob_faulty.append(StationaryFaultyRobot(p_faulty[i]))
    p[n_fault_free + i] = p_faulty[i]

for i in range(0, n_fault_free):
    rob_fault_free.append(FaultFreeRobot(p_fault_free[i], sensDist, alpha, velMax, delta, fixNeighbors=fixNeighbor))
    rob_fault_free[i].findNeighbors(p)

plt.figure(figsize=(2, 2))
t = 0

position_x_along_time = []
position_y_along_time = []

diff_nf_and_nf_approx = []

while t < 100:
    stop = True
    diff_nf_and_nf_approx_t = []

    plt.clf()
    # plt.grid(True, which='major')
    ax = plt.gca()
    ax.set_xlim(-box, box)
    ax.set_ylim(-box, box)
    # plt.xticks([0.3*i for i in range(-5,5, 1)])
    # plt.yticks([0.3*i for i in range(-5,5, 1)])
    plt.gca().set_aspect('equal', adjustable='box')
    # ax.grid(True)
    # for tic in ax.xaxis.get_major_ticks():
    #     tic.tick1On = tic.tick2On = False
    #     tic.label1On = tic.label2On = False
    # for tic in ax.yaxis.get_major_ticks():
    #     tic.tick1On = tic.tick2On = False
    #     tic.label1On = tic.label2On = False
    start, end = ax.get_xlim()
    # ax.xaxis.set_ticks(np.arange(start, end + 0.01, 0.2))
    # ax.yaxis.set_ticks(np.arange(start, end + 0.01, 0.2))

    ax.set_xticks([])
    ax.set_yticks([])
    ax.set_xticklabels([])
    ax.set_yticklabels([])
    for i in range(0, n_fault_free):
        for neighbor in rob_fault_free[i].neighbors:
            plt.plot([p_fault_free[i].x, p[neighbor].x], [p_fault_free[i].y, p[neighbor].y], linewidth=0.2,
                     color='gray')
    plot_point_set(p_faulty, color='r')  # faulty robots are plotted in red
    plot_point_set(p_fault_free, color='b')  # fault-free robots are plotted in blue
    plot_point_set([p_fault_free[4],p_fault_free[8]], color='yellow')  # fault-free robots are plotted in blue
    plt.pause(1)
    plt.savefig('./result/faultTolerantRendezvous/%s%d.eps' % (method, t))
    # end = input('Press enter to end the program.')
    t += 1
    print("iteration %d" % t)

    position_x_along_time.append([p.x for p in p_fault_free])
    position_y_along_time.append([p.y for p in p_fault_free])

    for i in range(0, n_fault_free):
        rob_fault_free[i].updatePos(p, method=method)
        if method == "tver":
            nf_approx_t = math.ceil(len(rob_fault_free[i].neighbors) / 4) - 1
        elif method == "center":
            nf_approx_t = math.ceil(len(rob_fault_free[i].neighbors) / 3) - 1
        else:
            nf_approx_t = 0
        nf_t = len([p for p in rob_fault_free[i].neighbors if p in range(n - n_faulty, n)])
        diff_nf_and_nf_approx_t.append(nf_approx_t - nf_t)

    for i in range(0, n_fault_free):
        p[i] = rob_fault_free[i].getPos()
        stop = stop and rob_fault_free[i].stop
    p_fault_free = p[:n_fault_free]
    p_faulty = p[n_fault_free:]

    diff_nf_and_nf_approx.append(diff_nf_and_nf_approx_t)

    # if stop:
    #    break

plt.figure(figsize=(5, 5))
ax = plt.gca()
if fixNeighbor:
    diff_set = set(diff_nf_and_nf_approx_t)
    diff_dict = {}
    for i in diff_set:
        diff_dict[i] = 0
        for j in diff_nf_and_nf_approx_t:
            if i == j:
                diff_dict[i] += 1
    for i in diff_dict.keys():
        ax.vlines(x=i, ymin=0, ymax=diff_dict[i], color='firebrick', alpha=0.7, linewidth=2)
        plt.draw()
    plt.xlabel(r'$\tilde{n}_{f_i} - n_{f_i}$', fontsize=24)
    plt.ylabel("Number", fontsize=24)
else:
    plt.plot(diff_nf_and_nf_approx)
    plt.draw()
    plt.xlabel("Iteration", fontsize=24)
    plt.ylabel(r'$\tilde{n}_{f_i} - n_{f_i}$', fontsize=24)
# ax.xaxis.set_ticks(np.arange(-5, 15, 5))
# ax.set_xticklabels(np.arange(-5, 15, 5))
plt.xticks(fontsize=15)
plt.yticks(fontsize=15)
ax.xaxis.set_major_locator(MaxNLocator(integer=True))
plt.tight_layout()
plt.savefig('./result/faultTolerantRendezvous/diff_nf_%s.eps' % (method))
#plt.show()

plt.figure(figsize=(5, 9))
plt.subplot(2, 1, 1)
plt.plot(position_x_along_time)
plt.draw()
plt.ylabel("Position (X)", fontsize=24)
ax = plt.gca()
# ax.xaxis.set_ticks(np.arange(0, 100, 10))
# ax.set_xticklabels(np.arange(0, 100, 10))
plt.xticks(fontsize=15)
plt.yticks(fontsize=15)
ax.xaxis.set_major_locator(MaxNLocator(integer=True))

plt.subplot(2, 1, 2)
plt.plot(position_y_along_time)
plt.draw()
plt.xlabel("Iteration", fontsize=24)
plt.ylabel("Position (Y)", fontsize=24)
ax = plt.gca()
# ax.xaxis.set_ticks(np.arange(0, 100, 10))
# ax.set_xticklabels(np.arange(0, 100, 10))
plt.xticks(fontsize=15)
plt.yticks(fontsize=15)
ax.xaxis.set_major_locator(MaxNLocator(integer=True))

plt.tight_layout()
plt.savefig('./result/faultTolerantRendezvous/positionChange_%s.eps' % (method))
#plt.show()

print("Total time used: %.2f s" % (time.time() - start_time))
# end = input('Press enter to end the program.')
