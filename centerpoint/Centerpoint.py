from copy import deepcopy
from shapely.geometry import Polygon
from centerpoint.HamCut.HamCut import *
from centerpoint.utils.GeoUtils import *
from centerpoint.utils.utils import *


class Centerpoint:
    def __init__(self, plot=False):
        self.point_set = None

        self.points_in_L = None
        self.points_not_in_L = None
        self.points_in_L_boundary = None

        self.points_in_U = None
        self.points_not_in_U = None
        self.points_in_D_boundary = None

        self.points_in_D = None
        self.points_not_in_D = None

        self.L_boundary_line = None
        self.U_boundary_line = None
        self.D_boundary_line = None
        self.R_boundary_line = None

        self.points_LU = None
        self.points_LD = None
        self.points_RU = None
        self.points_RD = None

        self.plot = plot

    def initialize(self, point_set):
        self.point_set = point_set
        self.n = len(self.point_set)
        self.np = math.ceil(self.n / 3)
        self.mp = math.ceil(self.n / 3) - math.ceil(self.n / 4)
        self.x_min, self.x_max = find_x_bounds(self.point_set)
        self.y_min, self.y_max = find_y_bounds(self.point_set)

    def getSafeCenterPoint(self, point_set):
        cp = []

        # determine if points are in the same line, if so, add small noise
        # point_set = set([(p.x, p.y) for p in self.point_set])
        self.point_set = deepcopy(point_set)
        leftmost = Point(min(self.point_set, key=lambda P: P.x).x, min(self.point_set, key=lambda P: P.x).y)
        rightmost = Point(max(self.point_set, key=lambda P: P.x).x, max(self.point_set, key=lambda P: P.x).y)
        l = LineString([leftmost, rightmost])
        oneLine = True
        for p in self.point_set:
            if p not in [leftmost, rightmost]:
                contain = l.contains(p)
                oneLine = oneLine and contain
        if oneLine:
            point_set = [Point(p.x+0.0001*random.random(), p.y+0.0001*random.random()) for p in point_set]

        for d in range(0, 3):
            self.point_set = deepcopy(point_set)
            l = Line(0.1 * (d+1), 0)
            p_trans = [point_transfer(p, 0, 0, l) for p in self.point_set]
            cp_trans = self.reduce_then_get_centerpoint(p_trans)
            cp_trans_back = point_transfer_back(cp_trans, 0, 0, l)
            cp.append(cp_trans_back)
        safe_point = Polygon([[p.x, p.y] for p in cp]).centroid
        return safe_point

    def reduce_then_get_centerpoint(self, point_set):
        self.point_set = point_set
        last_point_num = len(self.point_set)
        cur_point_num = 0
        #print("point number: %d" % last_point_num)
        while cur_point_num < last_point_num:
            last_point_num = len(self.point_set)
            self.initialize(self.point_set)
            try:
                self.find_L_boundary()
                self.find_U_boundary()
                self.find_D_boundary()
                self.find_R_boundary()
                self.find_intersections()
                self.replace_points()
                self.point_set = remove_repeat_points(self.point_set)
                cur_point_num = len(self.point_set)
                #print("point number: %d" % cur_point_num)
            except:
               pass
            cur_point_num = len(self.point_set)
        return self.brute_force_centerpoint()

    def brute_force_centerpoint(self):
        centerpoints = []
        self.point_set = remove_repeat_points(self.point_set)
        remaining_points = deepcopy(self.point_set)

        while len(self.point_set) > 6:
            p_corner = find_corner_points(self.point_set)[0:4]
            for p in p_corner:
                self.point_set.remove(p)
            if len(p_corner) == 4:
                Radon_point = get_Radon_point(p_corner[0], p_corner[1], p_corner[2], p_corner[3])
                self.point_set.append(Radon_point)

        # get r = math.ceil(n/3) Tverberg point for <= 6 points
        p_set = set([(p.x, p.y) for p in self.point_set])
        p_size = len(self.point_set)
        if p_size == 1:
            centerpoints.append(self.point_set[0])
        elif p_size <= 3:
            centerpoints.append(LineString([p for p in self.point_set]).centroid)
        elif p_size == 4:
            c = MultiPoint(self.point_set).convex_hull
            inside_points = [p for p in self.point_set if p.within(c)]
            if inside_points == []:
                centerpoints.append(
                    get_Radon_point(self.point_set[0], self.point_set[1], self.point_set[2], self.point_set[3]))
            else:
                centerpoints.append(inside_points[0])

        else:
            for i in range(0, p_size):
                for j in (j for j in range(0, p_size) if j > i):
                    for k in (k for k in range(0, p_size) if k > j):
                        subset1 = set([(p.x, p.y) for p in [self.point_set[i], self.point_set[j], self.point_set[k]]])
                        subset2 = p_set.difference(subset1)
                        p1 = Polygon([p for p in subset1])
                        try:
                            p2 = Polygon([p for p in subset2])
                        except:
                            try:
                                p2 = LineString([p for p in subset2])
                            except:
                                p2 = Point([p for p in subset2])

                        inter = p1.intersection(p2)
                        if not inter.is_empty:
                            cp = inter.centroid
                            centerpoints.append(cp)
                            break
                    else:
                        continue
                    break
                else:
                    continue
                break

        #for cp in centerpoints:
        #    print("Centerpoints: %.2f, %.2f" % (cp.x, cp.y))
        if self.plot:
            plt.clf()
            x_min, x_max = find_x_bounds(self.point_set)
            interval = Interval(x_min - 10, x_max + 10)
            y_min, y_max = find_y_bounds(self.point_set)
            prepare_axis(interval.l - 5, interval.r + 5, y_min - 5, y_max + 5)
            plot_point_set(remaining_points, color='b')
            plt.title('Centerpoint: %.2f, %.2f' % (centerpoints[0].x, centerpoints[0].y))
            # plot_point_set(remaining_points, color='b')
            plot_point(centerpoints[0], color='r')
            plt.pause(1)
            end = input('Press enter to the next step')
        return centerpoints[0]

    def find_L_boundary(self):
        leftmost_x = min(self.point_set, key=lambda P: P.x).x
        leftmost_y = min(self.point_set, key=lambda P: P.x).y
        leftmost = Point(leftmost_x, leftmost_y)

        slope = [(p.y - leftmost.y) / (p.x - leftmost.x) for p in self.point_set if
                 not (p == leftmost or p.x == leftmost.x)]

        equal_x_num = len([p for p in self.point_set if (p.x == leftmost.x and p.y > leftmost.y)])

        slope_np = findKthLargest(slope, self.np - 1 - equal_x_num)
        slope_np_index = slope.index(slope_np)

        index_before_num = len([p for p in self.point_set if
                                (p.x == leftmost.x and p.y >= leftmost.y and self.point_set.index(
                                    p) <= slope_np_index)])
        L_boundary_point = self.point_set[slope_np_index + index_before_num]

        self.L_boundary_line = Line((L_boundary_point.y - leftmost.y) / (L_boundary_point.x - leftmost.x),
                                    (leftmost.y * L_boundary_point.x - leftmost.x * L_boundary_point.y) / (
                                            L_boundary_point.x - leftmost.x))
        self.points_in_L = [p for p in self.point_set if
                            p.y > p.x * self.L_boundary_line.m + self.L_boundary_line.b + 1e-5]
        self.points_not_in_L = [p for p in self.point_set if
                                p.y < p.x * self.L_boundary_line.m + self.L_boundary_line.b - 1e-5]
        self.points_in_L_boundary = [p for p in self.point_set if
                                     (p.y <= p.x * self.L_boundary_line.m + self.L_boundary_line.b + 1e-5 and
                                      p.y >= p.x * self.L_boundary_line.m + self.L_boundary_line.b - 1e-5)]
        if self.plot:
            plt.clf()
            prepare_plot(self.point_set)
            plt.title('Find L boundary')
            plot_point(leftmost, color='r')
            plot_line(self.L_boundary_line, color='g')
            plot_point_set(self.points_in_L, color='r')
            plot_point_set(self.points_not_in_L, color='b')
            plot_point_set(self.points_in_L_boundary, color='g')
            plt.pause(1)
            # end = input('Press enter to the next step')

    def find_U_boundary(self):
        x0, y0 = self.points_in_L_boundary[0].x, self.points_in_L_boundary[0].y
        points_in_L_trans = [point_transfer(p, x0, y0, self.L_boundary_line) for p in self.points_in_L]
        points_not_in_L_trans = [point_transfer(p, x0, y0, self.L_boundary_line) for p in self.points_not_in_L]
        if not points_in_L_trans == [] and not points_not_in_L_trans == []:
            above = True if self.L_boundary_line.m >= 0 else False
            (red_points, blue_points) = (points_in_L_trans, points_not_in_L_trans) if points_in_L_trans[0].x < \
                                                                                      points_not_in_L_trans[0].x \
                else (points_not_in_L_trans, points_in_L_trans)
            (red_num_need, blue_num_need) = (
                self.mp - 1, self.np - self.mp - 1) if red_points == points_in_L_trans else (
                self.np - self.mp - 1, self.mp - 1)

            hamcut = HamCut()
            ham_cut_trans = hamcut.reduce_then_cut(red_points, blue_points, red_num_need, blue_num_need, above=above)
            ham_points_trans = [Point(self.x_min - 5, (self.x_min - 5) * ham_cut_trans.m + ham_cut_trans.b),
                                Point(self.x_max + 5, (self.x_max + 5) * ham_cut_trans.m + ham_cut_trans.b)]

            ham_points = [point_transfer_back(p, x0, y0, self.L_boundary_line) for p in ham_points_trans]

            self.U_boundary_line = line_over_two_points(ham_points[0], ham_points[1])

            self.points_in_U = [p for p in self.point_set if
                                p.y > p.x * self.U_boundary_line.m + self.U_boundary_line.b + 1e-5]
            self.points_not_in_U = [p for p in self.point_set if
                                    p.y < p.x * self.U_boundary_line.m + self.U_boundary_line.b - 1e-5]
            if len(self.points_in_U) > len(self.points_not_in_U):
                self.points_in_U, self.points_not_in_U = self.points_not_in_U, self.points_in_U

            self.points_in_U_boundary = [p for p in self.point_set if
                                         (p.y <= p.x * self.U_boundary_line.m + self.U_boundary_line.b + 1e-5 and
                                          p.y >= p.x * self.U_boundary_line.m + self.U_boundary_line.b - 1e-5)]
            if self.plot:
                plt.title('Find U boundary')
                plot_line(self.U_boundary_line, color='k')
                plot_point_set(self.points_in_U, color='r')
                plot_point_set(self.points_not_in_U, color='b')
                plot_point_set(self.points_in_U_boundary, color='g')
                plt.pause(1)
                # end = input('Press enter to the next step')

    def find_D_boundary(self):
        x0, y0 = self.points_in_L_boundary[0].x, self.points_in_L_boundary[0].y
        points_in_L_trans = [point_transfer(p, x0, y0, self.L_boundary_line) for p in self.points_in_L]
        points_not_in_L_trans = [point_transfer(p, x0, y0, self.L_boundary_line) for p in self.points_not_in_L]
        if not points_in_L_trans == [] and not points_not_in_L_trans == []:
            above = False if self.L_boundary_line.m >= 0 else True
            (red_points, blue_points) = (points_in_L_trans, points_not_in_L_trans) if points_in_L_trans[0].x < \
                                                                                      points_not_in_L_trans[0].x \
                else (points_not_in_L_trans, points_in_L_trans)
            (red_num_need, blue_num_need) = (
                self.mp - 1, self.np - self.mp - 1) if red_points == points_in_L_trans else (
                self.np - self.mp - 1, self.mp - 1)

            hamcut = HamCut()
            ham_cut_trans = hamcut.reduce_then_cut(red_points, blue_points, red_num_need, blue_num_need, above=above)

            ham_points_trans = [Point(self.x_min - 5, (self.x_min - 5) * ham_cut_trans.m + ham_cut_trans.b),
                                Point(self.x_max + 5, (self.x_max + 5) * ham_cut_trans.m + ham_cut_trans.b)]

            ham_points = [point_transfer_back(p, x0, y0, self.L_boundary_line) for p in ham_points_trans]

            self.D_boundary_line = line_over_two_points(ham_points[0], ham_points[1])

            # slope_reverse = 1 if y0 > x0 * self.D_boundary_line.m + self.D_boundary_line.b + 1e-5 else -1
            self.points_in_D = [p for p in self.point_set if
                                p.y > p.x * self.D_boundary_line.m + self.D_boundary_line.b + 1e-5]
            self.points_not_in_D = [p for p in self.point_set if
                                    p.y < p.x * self.D_boundary_line.m + self.D_boundary_line.b - 1e-5]
            if len(self.points_in_D) > len(self.points_not_in_D):
                self.points_in_D, self.points_not_in_D = self.points_not_in_D, self.points_in_D
            points_in_D_boundary = [p for p in self.point_set if
                                    (p.y <= p.x * self.D_boundary_line.m + self.D_boundary_line.b + 1e-5 and
                                     p.y >= p.x * self.D_boundary_line.m + self.D_boundary_line.b - 1e-5)]
            if self.plot:
                plt.title('Find D boundary')
                plot_line(self.U_boundary_line, color='g')
                plot_line(self.D_boundary_line, color='k')
                plot_point_set(self.points_in_D, color='r')
                plot_point_set(self.points_not_in_D, color='b')
                plot_point_set(points_in_D_boundary, color='g')
                plt.pause(1)
                # end = input('Press enter to the next step')

    def find_R_boundary(self):
        x0, y0 = self.points_in_U_boundary[0].x, self.points_in_U_boundary[0].y
        points_in_U_trans = [point_transfer(p, x0, y0, self.U_boundary_line) for p in self.points_in_U]
        points_not_in_U_trans = [point_transfer(p, x0, y0, self.U_boundary_line) for p in self.points_not_in_U]

        # above = True if self.U_boundary_line.m < 0 else False
        if not points_in_U_trans == [] and not points_not_in_U_trans == []:
            (red_points, blue_points) = (points_in_U_trans, points_not_in_U_trans) if points_in_U_trans[0].x < \
                                                                                      points_not_in_U_trans[0].x \
                else (points_not_in_U_trans, points_in_U_trans)
            (red_num_need, blue_num_need) = (
                self.mp - 1, self.np - self.mp - 1) if red_points == points_in_U_trans else (
                self.np - self.mp - 1, self.mp - 1)

            hamcut = HamCut()
            ham_cut_trans = hamcut.reduce_then_cut(red_points, blue_points, red_num_need, blue_num_need, above=False)

            ham_points_trans = [Point(self.x_min - 5, (self.x_min - 5) * ham_cut_trans.m + ham_cut_trans.b),
                                Point(self.x_max + 5, (self.x_max + 5) * ham_cut_trans.m + ham_cut_trans.b)]

            ham_points = [point_transfer_back(p, x0, y0, self.U_boundary_line) for p in ham_points_trans]

            self.R_boundary_line = line_over_two_points(ham_points[0], ham_points[1])
            self.points_in_R = [p for p in self.point_set if
                                p.y > p.x * self.R_boundary_line.m + self.R_boundary_line.b + 1e-5]
            self.points_not_in_R = [p for p in self.point_set if
                                    p.y < p.x * self.R_boundary_line.m + self.R_boundary_line.b - 1e-5]
            if len(self.points_in_R) > len(self.points_not_in_R):
                self.points_in_R, self.points_not_in_R = self.points_not_in_R, self.points_in_R

            points_in_R_boundary = [p for p in self.point_set if
                                    (p.y <= p.x * self.R_boundary_line.m + self.R_boundary_line.b + 1e-5 and
                                     p.y >= p.x * self.R_boundary_line.m + self.R_boundary_line.b - 1e-5)]
            if self.plot:
                plt.title('Find R boundary')
                plot_line(self.U_boundary_line, color='g')
                plot_line(self.D_boundary_line, color='g')
                plot_line(self.R_boundary_line, color='k')
                plot_point_set(self.points_in_R, color='r')
                plot_point_set(self.points_not_in_R, color='b')
                plot_point_set(points_in_R_boundary, color='g')
                plt.pause(1)
                # end = input('Press enter to the next step')

    def find_intersections(self):
        set_L = set([(p.x, p.y) for p in self.points_in_L])
        set_U = set([(p.x, p.y) for p in self.points_in_U])
        set_R = set([(p.x, p.y) for p in self.points_in_R])
        set_D = set([(p.x, p.y) for p in self.points_in_D])
        set_all_points = set([(p.x, p.y) for p in self.point_set])
        set_LU = set_L.intersection(set_U)
        set_LD = set_L.intersection(set_D)
        set_RU = set_R.intersection(set_U)
        set_RD = set_R.intersection(set_D)
        set_points_in_intersection = set_LU.union(set_LD).union(set_RU).union(set_RD)
        set_points_not_in_intersection = set_all_points.difference(set_points_in_intersection)
        self.points_LU = [Point(p[0], p[1]) for p in list(set_LU)]
        self.points_LD = [Point(p[0], p[1]) for p in list(set_LD)]
        self.points_RU = [Point(p[0], p[1]) for p in list(set_RU)]
        self.points_RD = [Point(p[0], p[1]) for p in list(set_RD)]
        self.points_not_in_intersection = [Point(p[0], p[1]) for p in list(set_points_not_in_intersection)]

        if self.plot:
            plt.title('Find P_LU, P_LD, P_RU, P_RD')
            plot_point_set(self.point_set, color='b')
            plot_point_set(self.points_LU, color='r')
            plot_point_set(self.points_LD, color='g')
            plot_point_set(self.points_RU, color='m')
            plot_point_set(self.points_RD, color='k')
            plt.pause(1)
            end = input('Press enter to the next step')

    def replace_points(self):
        Radon_point_set = []
        while self.points_LU != [] and self.points_LD != [] and self.points_RU != [] and self.points_RD != []:
            p_LU, p_LD, p_RU, p_RD = self.points_LU.pop(), self.points_LD.pop(), self.points_RU.pop(), self.points_RD.pop()
            Radon_point = get_Radon_point(p_LU, p_RU, p_RD, p_LD)
            Radon_point_set.append(Radon_point)
            if not math.isnan(Radon_point.x) and not math.isnan(Radon_point.y):
                self.points_not_in_intersection.append(Radon_point)

        set_LU = set([(p.x, p.y) for p in self.points_LU])
        set_LD = set([(p.x, p.y) for p in self.points_LD])
        set_RU = set([(p.x, p.y) for p in self.points_RU])
        set_RD = set([(p.x, p.y) for p in self.points_RD])
        set_points_not_in_intersection = set([(p.x, p.y) for p in self.points_not_in_intersection])
        set_all_points = set_points_not_in_intersection.union(set_LU).union(set_LD).union(set_RU).union(set_RD)
        self.point_set = [Point(p[0], p[1]) for p in list(set_all_points)]

        if self.plot:
            plt.clf()
            prepare_plot(self.point_set)
            plt.title('replace Q by their Radon point')
            plot_line(self.L_boundary_line, color='g')
            plot_line(self.U_boundary_line, color='g')
            plot_line(self.D_boundary_line, color='g')
            plot_line(self.R_boundary_line, color='g')
            plot_point_set(Radon_point_set, color='r')
            plt.pause(1)
            end = input('Press enter to the next step')