#!/usr/bin/python3

import time

from centerpoint.Centerpoint import *

random.seed(1)


def plotResult(point_set, cp):
    plt.clf()
    x_min, x_max = find_x_bounds(point_set)
    interval = Interval(x_min - 10, x_max + 10)
    y_min, y_max = find_y_bounds(point_set)
    prepare_axis(interval.l - 5, interval.r + 5, y_min - 5, y_max + 5)
    plot_point_set(point_set, color='b')
    plot_point(cp, color='r')
    plt.pause(1)
    end = input('Press enter to the next step.')


if __name__ == '__main__':
    n = 1000  # total number of points
    plot = True
    start_time = time.time()
    point_set = random_point_set(n, lower=-100, upper=100)
    cp = Centerpoint(plot=plot)
    centerpoint = cp.reduce_then_get_centerpoint(point_set)
    #centerpoint = cp.getSafeCenterPoint(point_set)
    print("Centerpoints: %.2f, %.2f" % (centerpoint.x, centerpoint.y))
    print("Total time used for %d points is: %.2f s" % (n, time.time() - start_time))

    plotResult(point_set, centerpoint)
