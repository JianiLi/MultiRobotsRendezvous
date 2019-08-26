from centerpoint.utils.GeoUtils import *
from centerpoint.utils.PlotUtils import *
from centerpoint.utils.utils import *


class HamCut:
    def __init__(self):
        self.red_points = None
        self.blue_points = None
        self.red_num = None
        self.blue_num = None

        # all points
        self.all_points = None

        # red duals are with negative slope and blue duals are with positive slope
        self.red_duals = None
        self.blue_duals = None
        self.remaining_blue_duals = None

        self.num_need_red = None
        self.num_need_blue = None
        self.above = True

        self.ham_cut = None
        self.ham_point = None

        self.plot = None

    def reduce_then_cut(self, red_points, blue_points, num_need_red, num_need_blue, above=True, plot=False):
        self.red_points = red_points
        self.blue_points = blue_points
        self.all_points = self.red_points + self.blue_points
        self.red_num = len(self.red_points)
        self.blue_num = len(self.blue_points)
        self.above = above
        if self.above == True:
            self.num_need_red = num_need_red
            self.num_need_blue = num_need_blue
        else:
            self.num_need_red = self.red_num - 1 - num_need_red
            self.num_need_blue = self.blue_num - 1 - num_need_blue
        self.plot = plot

        # red duals are with negative slope and blue duals are with positive slope
        self.red_duals = [compute_dual_line(p) for p in self.red_points]
        self.blue_duals = [compute_dual_line(p) for p in self.blue_points]
        self.remaining_blue_duals = self.blue_duals

        query_positive_line = random.choice(self.blue_duals)
        if self.plot:
            prepare_plot(self.all_points)
            plot_points_and_duals(self.red_points, self.blue_points, self.red_duals, self.blue_duals, t=0.5)
            plt.pause(1)
            end = input('Press enter to the next step.')
            plt.clf()
            x_min, x_max = find_x_bounds(self.all_points)
            interval = Interval(x_min - 10, x_max + 10)
            y_min, y_max = find_y_bounds(self.all_points)
            prepare_axis(interval.l - 5, interval.r + 5, y_min - 5, y_max + 5)
            for r in self.red_duals:
                plot_line(r, color='r')
            plot_line(query_positive_line, color='b')
            plt.pause(1)
            end = input('Press enter to the next step.')

        return self.query_line(query_positive_line)

    def query_line(self, query_line):
        intersections_with_query_line = [Intersection(red_dual, query_line) for red_dual in
                                         self.red_duals]
        kth_intersection_x = findKthLargest([inter.x for inter in intersections_with_query_line],
                                            self.red_num + 1 - self.num_need_red - 1)
        kth_intersection_y = intersections_with_query_line[
            [inter.x for inter in intersections_with_query_line].index(kth_intersection_x)].y
        if self.plot:
            plt.clf()
            plt.title('Find kth intersection')
            x_min, x_max = find_x_bounds(self.all_points)
            interval = Interval(x_min - 10, x_max + 10)
            y_min, y_max = find_y_bounds(self.all_points)
            prepare_axis(interval.l - 5, interval.r + 5, y_min - 5, y_max + 5)
            for r in self.red_duals:
                plot_line(r, color='r')
            plot_line(query_line, color='b')
            plot_point(Point(kth_intersection_x, kth_intersection_y), color='g')
            plt.pause(1)
            end = input('Press enter to the next step.')

        positive_duals_below_point = [l for l in self.blue_duals if
                                      kth_intersection_y > l.m * kth_intersection_x + l.b and not l == query_line]
        if self.plot:
            plt.title('Blue duals below intersection')
            for b in positive_duals_below_point:
                plot_line(b, color='b')
            plt.pause(1)
            end = input('Press enter to the next step.')

        if len(positive_duals_below_point) == self.num_need_blue:
            self.ham_point = Point(kth_intersection_x, kth_intersection_y)
            self.ham_cut = compute_dual_line(self.ham_point)
            if self.plot:
                plt.clf()
                x_min, x_max = find_x_bounds(self.all_points)
                interval = Interval(x_min - 10, x_max + 10)
                y_min, y_max = find_y_bounds(self.all_points)
                prepare_axis(interval.l - 5, interval.r + 5, y_min - 5, y_max + 5)
                plot_point_set(self.red_points, color='r')
                plot_point_set(self.blue_points, color='b')
                plot_line(self.ham_cut, color='g')
                plt.pause(1)
                end = input('Press enter to the next step.')
            return self.ham_cut
        elif len(positive_duals_below_point) > self.num_need_blue:
            self.remaining_blue_duals = [l for l in self.remaining_blue_duals if l in positive_duals_below_point]
            if len(self.remaining_blue_duals) == 1:
                return self.query_line(self.remaining_blue_duals[0])
            else:
                y_vals = [l.m * kth_intersection_x + l.b for l in self.remaining_blue_duals]
                y_vals_med = findKthLargest(y_vals, math.floor(len(y_vals) / 2))
                dual_med = self.remaining_blue_duals[y_vals.index(y_vals_med)]
                return self.query_line(dual_med)
        else:
            self.remaining_blue_duals = [l for l in self.remaining_blue_duals if
                                         l not in positive_duals_below_point and not l == query_line]
            if len(self.remaining_blue_duals) == 1:
                return self.query_line(self.remaining_blue_duals[0])
            else:
                y_vals = [l.m * kth_intersection_x + l.b for l in self.remaining_blue_duals]
                y_vals_med = findKthLargest(y_vals, math.floor(len(y_vals) / 2))
                dual_med = self.remaining_blue_duals[y_vals.index(y_vals_med)]
                return self.query_line(dual_med)


if __name__ == '__main__':
    random.seed(1)
    n_red = 5000
    n_blue = 5000
    red_points = []
    blue_points = []
    num_need_red = int(n_red / 12)
    num_need_blue = int(n_blue / 4)
    above = True
    plot = False
    for i in range(n_red):
        x = random.uniform(-5, 0)
        y = random.uniform(-5, 5)
        red_points.append(Point(x, y))
    for i in range(n_blue):
        x = random.uniform(0, 5)
        y = random.uniform(-5, 5)
        blue_points.append(Point(x, y))

    hamcut = HamCut()
    cut = hamcut.reduce_then_cut(red_points, blue_points, num_need_red, num_need_blue, above=above, plot=plot)
    print(cut.m, cut.b)

    plt.clf()
    x_min, x_max = find_x_bounds(hamcut.all_points)
    interval = Interval(x_min - 10, x_max + 10)
    y_min, y_max = find_y_bounds(hamcut.all_points)
    prepare_axis(interval.l - 5, interval.r + 5, y_min - 5, y_max + 5)
    plot_point_set(hamcut.red_points, color='r')
    plot_point_set(hamcut.blue_points, color='b')
    plot_line(hamcut.ham_cut, color='g')
    plt.pause(1)
    end = input('Press enter to the next step.')
