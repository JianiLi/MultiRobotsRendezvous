import math
import numpy as np
import random
from shapely.geometry import Point
from shapely.geometry import MultiPoint
from shapely.geometry import LineString

from centerpoint.utils.PlotUtils import *
from centerpoint.utils.utils import *

class Line:
    def __init__(self, m, b):
        self.m = m
        self.b = b


class LineSegment:
    def __init__(self, point1, point2):
        self.p1 = point1
        self.p2 = point2

    def __str__(self):
        return 'LineSegment ({}, {})'.format(self.p1, self.p2)


class Interval:
    def __init__(self, l, r):
        self.l = l
        self.r = r
        assert l <= r

    def __str__(self):
        return 'Interval from {} to {}'.format(self.l, self.r)

    def __len__(self):
        return int(self.r - self.l)


class Intersection:
    def __init__(self, line1, line2):
        if line1.m == line2.m:
            self.x = np.inf
            self.y = np.inf
        else:
            self.x = (line2.b - line1.b) / (line1.m - line2.m)
            self.y = line1.m * self.x + line1.b




def remove_repeat_points(point_set):
    points = [(p.x, p.y) for p in point_set if not p.x == np.inf and not p.y == np.inf]
    points = list(set(points))
    point_set = [Point(p[0], p[1]) for p in points]

    return point_set


def find_corner_points(point_set):
    convex_hull = MultiPoint(point_set).convex_hull
    corner_points = [p for p in point_set if not p.within(convex_hull)]
    return corner_points


def line_over_two_points(p1, p2):
    if p1.x == p2.x:
        return Line(np.inf, 0)
    else:
        k = (p2.y - p1.y) / (p2.x - p1.x)
        b = -p1.x * ((p2.y - p1.y) / (p2.x - p1.x)) + p1.y
    return Line(k, b)


def find_x_bounds(point_set):
    min_x = min(point_set, key=lambda P: P.x).x
    max_x = max(point_set, key=lambda P: P.x).x
    return min_x, max_x


def find_y_bounds(point_set):
    min_y = min(point_set, key=lambda P: P.y).y
    max_y = max(point_set, key=lambda P: P.y).y
    return min_y, max_y


def compute_dual_line(P):
    """Compute dual of a point

    Arguments:
        P {shapely.geometry.Point}
    """
    return Line(P.x, -P.y)


def random_point_set(n, lower=-10, upper=10):
    points = []
    assert lower <= upper
    for i in range(n):
        x = random.uniform(lower, upper)
        y = random.uniform(lower, upper)
        points.append(Point(x, y))
    return points


def get_Radon_point(p1, p2, p3, p4):
    point_set = [p1,p2,p3,p4]
    convex_hull = MultiPoint([p1,p2,p3,p4]).convex_hull
    if p1.within(convex_hull):
        return p1
    elif p2.within(convex_hull):
        return p2
    elif p3.within(convex_hull):
        return p3
    elif p4.within(convex_hull):
        return p4
    else:
        for a, b in zip([(p1,p3),(p1,p2),(p1,p4)], [(p2,p4),(p3,p4),(p2,p3)]):
            line1 = line_over_two_points(a[0],a[1])
            line2 = line_over_two_points(b[0],b[1])
            Radon_point = Point(Intersection(line1, line2).x,Intersection(line1, line2).y)
            if Radon_point.within(convex_hull):
                return Radon_point
        X = [p.x for p in point_set]
        med_x = findKthLargest(X, 2)
        index = X.index(med_x)
        return Point(point_set[index].x, point_set[index].y)



def point_transfer(p, x0, y0, line):
    x = p.x
    y = p.y
    k = line.m
    b = line.b
    sin_angle = -1 / (math.sqrt(k ** 2 + 1)) if k > 0 else 1 / (math.sqrt(k ** 2 + 1))
    cos_angle = k / (math.sqrt(k ** 2 + 1)) if k > 0 else -k / (math.sqrt(k ** 2 + 1))
    x_prime = (x - x0) * cos_angle + (y - y0) * sin_angle
    y_prime = (y - y0) * cos_angle - (x - x0) * sin_angle
    return Point(x_prime, y_prime)


def point_transfer_back(p_prime, x0, y0, line):
    x_prime = p_prime.x
    y_prime = p_prime.y
    k = line.m
    b = line.b
    sin_angle = -1 / (math.sqrt(k ** 2 + 1)) if k > 0 else 1 / (math.sqrt(k ** 2 + 1))
    cos_angle = k / (math.sqrt(k ** 2 + 1)) if k > 0 else -k / (math.sqrt(k ** 2 + 1))
    x = x_prime * cos_angle - y_prime * sin_angle + x0
    y = y_prime * cos_angle + x_prime * sin_angle + y0
    return Point(x, y)


def test_point_transfer():
    '''
    if point transfer and transfer back functions are correct,
    the black and red points should overlap.
    '''
    k = 0.1
    b = 0
    l = Line(k, b)
    x0 = 2
    y0 = k * x0 + b
    coord = Point(x0, y0)
    l_pend = Line(-1 / k, 1 / k * coord.x + coord.y)

    sin_angle = -1 / (math.sqrt(k ** 2 + 1)) if k > 0 else 1 / (math.sqrt(k ** 2 + 1))
    cos_angle = k / (math.sqrt(k ** 2 + 1)) if k > 0 else -k / (math.sqrt(k ** 2 + 1))

    plt.ion()
    plt.show()
    prepare_axis()
    plot_line(l)
    plot_line(l_pend)

    point_set = random_point_set(10, lower=-10, upper=10)
    plot_point_set(point_set, color='k')
    P = []
    P_trans = []

    for p in point_set:
        p_trans = point_transfer(p, x0, y0, l)
        P_trans.append(p_trans)
        p_trans_back = point_transfer_back(p_trans, x0, y0, l)
        P.append(p_trans_back)

    # plot_point_set(P_trans, color='b')
    plot_point_set(P, color='r')

    plt.pause(1)
    end = input('Press enter to end the next step')


if __name__ == '__main__':
    test_point_transfer()
