import numpy as np
from matplotlib import pyplot as plt


def prepare_plot(point_set):
    plt.ion()
    plt.show()
    plt.title('Points')
    x_min, x_max = find_x_bounds(point_set)
    interval = Interval(x_min - 10, x_max + 10)
    y_min, y_max = find_y_bounds(point_set)
    prepare_axis(interval.l - 5, interval.r + 5, y_min - 5, y_max + 5)
    plot_point_set(point_set)
    plt.pause(1)


def find_x_bounds(point_set):
    min_x = min(point_set, key=lambda P: P.x).x
    max_x = max(point_set, key=lambda P: P.x).x
    return min_x, max_x


def find_y_bounds(point_set):
    min_y = min(point_set, key=lambda P: P.y).y
    max_y = max(point_set, key=lambda P: P.y).y
    return min_y, max_y


class Interval:
    def __init__(self, l, r):
        self.l = l
        self.r = r
        assert l <= r

    def __str__(self):
        return 'Interval from {} to {}'.format(self.l, self.r)

    def __len__(self):
        return int(self.r - self.l)


def prepare_axis(min_x=-10, max_x=10, min_y=-10, max_y=10):
    plt.grid(True, which='major')
    ax = plt.gca()
    min_x = int(min_x)
    max_x = int(max_x)
    min_y = int(min_y)
    max_y = int(max_y)
    ax.set_xlim(min_x, max_x)
    ax.set_ylim(min_y, max_y)
    plt.xticks(np.arange(min(np.array(ax.get_xlim())), max(np.array(ax.get_xlim())) + 1, int((max_x-min_x)/10)))
    plt.yticks(np.arange(min(np.array(ax.get_ylim())), max(np.array(ax.get_ylim())) + 1, int((max_y-min_y)/10)))
    plt.gca().set_aspect('equal', adjustable='box')
    plt.axhline(0, color='black')
    plt.axvline(0, color='black')


def plot_line(L, linestyle='--', color='b'):
    axes = plt.gca()
    x_vals = np.array(axes.get_xlim())
    y_vals = L.b + L.m * x_vals
    plt.plot(x_vals, y_vals, ls=linestyle, color=color)
    plt.draw()


def plot_vertical_line(x, linestyle=':', color='g', linewidth=5):
    plt.axvline(x=x, ls=linestyle, color=color, linewidth=linewidth)


def plot_line_segment(L, linestyle='-', color='b'):
    plt.plot([L.p1.x, L.p2.x], [L.p1.y, L.p2.y], ls=linestyle, color=color, linewidth=4)


def plot_point(P, marker='o', color='b', size=4):
    plt.plot(P.x, P.y, marker=marker, color=color, markersize=size, markeredgecolor='k', markeredgewidth=0.1)
    plt.draw()


def plot_points_and_duals(red_points, blue_points, red_duals, blue_duals, t=0.5):
    for p, d in zip(red_points, red_duals):
        plot_point(p, color='r')
        plt.draw()
        # plt.pause(t)
        plot_line(d, color='r')
        plt.draw()
        # plt.pause(t)
    for p, d in zip(blue_points, blue_duals):
        plot_point(p, color='b')
        plt.draw()
        # plt.pause(t)
        plot_line(d, color='b')
        plt.draw()
        # plt.pause(t)


def plot_point_set(point_set, color='b'):
    for p in point_set:
        plot_point(p, color=color)


def plot_interval(I, linestyle=':', color='g'):
    plot_vertical_line(I.l, linestyle=linestyle, color=color, linewidth=2)
    plot_vertical_line(I.r, linestyle=linestyle, color=color, linewidth=2)
