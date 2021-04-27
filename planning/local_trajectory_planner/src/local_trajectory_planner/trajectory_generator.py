 
"""
State lattice planner with model predictive trajectory generator
author: Atsushi Sakai (@Atsushi_twi)
- lookuptable.csv is generated with this script: https://github.com/AtsushiSakai/PythonRobotics/blob/master/PathPlanning/ModelPredictiveTrajectoryGenerator/lookuptable_generator.py
Ref:
- State Space Sampling of Feasible Motions for High-Performance Mobile Robot Navigation in Complex Environments http://citeseerx.ist.psu.edu/viewdoc/download?doi=10.1.1.187.8210&rep=rep1&type=pdf
"""
import itertools
import heapq
import os
import sys
from matplotlib import pyplot as plt
import numpy as np
import math
from sklearn.neighbors import KDTree
from .pycubicspline import *


def calc_2d_spline_interpolation(x, y, num=100):
    """
    Calc 2d spline course with interpolation
    :param x: interpolated x positions
    :param y: interpolated y positions
    :param num: number of path points
    :return:
        - x     : x positions
        - y     : y positions
        - yaw   : yaw angle list
        - k     : curvature list
        - s     : Path length from start point
    """
    sp = Spline2D(x, y)
    s = np.linspace(0, sp.s[-1], num+1)[:-1]

    r_x, r_y, r_yaw, r_k = [], [], [], []
    for i_s in s:
        ix, iy = sp.calc_position(i_s)
        r_x.append(ix)
        r_y.append(iy)
        r_yaw.append(sp.calc_yaw(i_s))
        r_k.append(sp.calc_curvature(i_s))

    travel = np.cumsum([np.hypot(dx, dy) for dx, dy in zip(np.diff(r_x), np.diff(r_y))]).tolist()
    travel = np.concatenate([[0.0], travel])

    return r_x, r_y, r_yaw, r_k, travel


def calc_trajectory(start_pos, end_pos): 
    start_loc = start_pos[:2]
    end_loc = end_pos[ :2]

    start_coeff = 0.5
    end_coeff = 0.5
    ds = np.array([1, 0]) * start_coeff
    de = np.array([-1, 0]) * end_coeff

    R1 = generate_rotation_matrix(start_pos[2])
    R2 = generate_rotation_matrix(end_pos[2])

    start_loc_2 = start_loc + np.dot(R1, ds)
    end_loc_2 = end_loc + np.dot(R2, de)

    x = [start_loc[0], start_loc_2[0], end_loc_2[0], end_loc[0]]
    y = [start_loc[1], start_loc_2[1], end_loc_2[1], end_loc[1]]

    x_1, y_1, yaw, k, travel = calc_2d_spline_interpolation(x, y)

    return x_1, y_1

def plot_trajectories(X, Y):

    for x, y in zip(X, Y):

        # plt.subplots(1)
        plt.plot(x, y, label="trajectory")
        # plt.plot(x, y, "xb", label="input")
        # plt.plot(x, y, "-r", label="spline")
        plt.grid(True)
        plt.axis("equal")
        plt.xlabel("x[m]")
        plt.ylabel("y[m]")
        plt.legend()

    # plt.subplots(1)
    # plt.plot(travel, [math.degrees(i_yaw) for i_yaw in yaw], "-r", label="yaw")
    # plt.grid(True)
    # plt.legend()
    # plt.xlabel("line length[m]")
    # plt.ylabel("yaw angle[deg]")

    # plt.subplots(1)
    # plt.plot(travel, k, "-r", label="curvature")
    # plt.grid(True)
    # plt.legend()
    # plt.xlabel("line length[m]")
    # plt.ylabel("curvature [1/m]")

    plt.show()


def generate_rotation_matrix(theta):
    c, s = np.cos(theta), np.sin(theta)
    R = np.array(((c, -s), (s, c)))
    return R

def generate_trajectories(robot_pos, target_pos, point_dist, point_count):
    points = [target_pos]
    
    target_yaw = target_pos[2]
    target_loc = np.array(target_pos[ :2])

    x, y = calc_trajectory(robot_pos, target_pos)
    X, Y = [x], [y]

    for i in range(point_count):
        dp1 = [0, point_dist * (i+1)]
        dp2 = [0, -point_dist * (i+1)]

        # c, s = np.cos(target_yaw), np.sin(target_yaw)
        R = generate_rotation_matrix(target_yaw)

        p1 = target_loc + np.dot(R, dp1)
        p2 = target_loc + np.dot(R, dp2)

        pos1 = np.zeros_like(robot_pos)
        pos1[:2] = p1
        pos1[2] = target_yaw

        pos2 = np.zeros_like(robot_pos)
        pos2[:2] = p2
        pos2[2] = target_yaw

        points.extend([pos1, pos2])

        x, y = calc_trajectory(robot_pos, pos1)
        X.append(x)
        Y.append(y)

        x, y = calc_trajectory(robot_pos, pos2)
        X.append(x)
        Y.append(y)

    plot_trajectories(X, Y)


    print(points)


def find_nearest_idx(array, value):
    array = np.asarray(array)
    idx = (np.linalg.norm(array - value, axis=1)).argmin()
    # print(idx)
    # print(np.linalg.norm(array - value, axis=1))
    # print(np.linalg.norm(array - value, axis=0))
    # raise
    return idx


eps = 1e-5

class Trajectory(object):
    def __init__(self, points):
        self.points = list()
        self.points.append(points[0][:2].copy())

        for i in range(len(points) - 1):
            start_loc = points[i][:2]
            yaw1 = points[i][2]

            end_loc = points[i + 1][:2]
            yaw2 = points[i + 1][2]

            start_coeff = 0.15
            end_coeff = 0.15
            ds = np.array([1, 0]) * start_coeff
            de = np.array([-1, 0]) * end_coeff

            R1 = generate_rotation_matrix(yaw1)
            R2 = generate_rotation_matrix(yaw2)

            start_loc_2 = start_loc + np.dot(R1, ds)
            end_loc_2 = end_loc + np.dot(R2, de)

            self.points.extend([start_loc_2, end_loc_2, end_loc])
        self.path = None
        self.x_spline, self.y_spline, self.yaw, self.k, self.travel = None, None, None, None, None
        # self.points = points

    def get_trajectory(self):
        if self.x_spline is None:
            self.calc_trajctory()

        return self.x_spline, self.y_spline

    def calc_trajctory(self):
        x = [p[0] for p in self.points]
        y = [p[1] for p in self.points]

        x_spline, y_spline, yaw, k, travel = calc_2d_spline_interpolation(x, y, num=10)
        self.x_spline, self.y_spline, self.yaw, self.k, self.travel = x_spline, y_spline, yaw, k, travel
        self.path = np.column_stack((self.x_spline, self.y_spline))
        # self.path = zip(self.x_spline, self.y_spline)

    def get_travel_score(self):
        return np.linalg.norm(np.array(self.travel)) + eps

    def get_distance_to_path_score(self, target_path):
        tree = KDTree(self.path)
        neighbor_dists, neighbor_indices = tree.query(target_path)
        return np.sum(neighbor_dists) + eps

    def get_curvature_score(self):
        return np.sum(np.fabs(self.k)) + eps

    def get_poses(self):
        poses = list()
        for x, y, yaw in zip(self.x_spline, self.y_spline, self.yaw):
            poses.append((x, y, yaw))

        return poses


class LocalTrajectoryGenerator(object):
    def __init__(self):
        self.N = 3.0
        self.SUBPOINTS = 3
        self.SUBPOINTS_DIST = .50

    def find_loc_trajectory(self, path, robot_pos):
        u"""
        Args:
            path (np.ndarray).
            robot_pos (np.ndarray(3x1)).
        """
        nearest_point_index = find_nearest_idx(path, robot_pos[:2])
        n, n_next = self.find_next_points(path, nearest_point_index)

        n_subpoints = self.create_sub_points(path, n)
        n_next_subpoints = None

        if n_next is not None:
            n_next_subpoints = self.create_sub_points(path, n_next)

        trajectories = self.calc_trajectories(robot_pos, n_subpoints, n_next_subpoints)

        # self.plot(path, robot_pos, trajectories)

        best_trajectory = self.get_best_one(trajectories, path)
        return best_trajectory.get_poses()
        # self.plot(path, robot_pos, [best_trajectory])


    def get_best_one(self, trajectories, target_path):
        scores = []
        for trajectory in trajectories:
            ts = trajectory.get_travel_score()
            ds = trajectory.get_distance_to_path_score(target_path)
            cs = trajectory.get_curvature_score()

            heapq.heappush(scores, (ts * ds * cs, trajectory))

        # print([score[0] for score in scores])

        return scores[0][1]


    def plot(self, path, robot_pos, trajectories):
        plt.plot(path[:, 0], path[:, 1], 'x', label='Path')
        plt.plot(robot_pos[0], robot_pos[1], 'o', label="Robot")
        for trajectory in trajectories:
            x, y = trajectory.get_trajectory()
            plt.plot(x, y)

        plt.grid(True)
        plt.axis("equal")
        plt.xlabel("x[m]")
        plt.ylabel("y[m]")
        plt.legend()

        plt.show()


    def calc_trajectories(self, rob_pos, n_subpoints, n_next_subpoints):
        trajectories = list()

        n_next_subpoints = [] if n_next_subpoints is None else n_next_subpoints

        for comb in itertools.product([rob_pos], n_subpoints, n_next_subpoints):
            t = Trajectory(comb)
            trajectories.append(t)
            t.calc_trajctory()
        return trajectories


    def find_next_points(self, path, robot_nearest_point):
        u"""
        Args:
            path (np.ndarray).
            robot_nearest_point (int).
        """
        point_index = robot_nearest_point
        path_len = path.shape[0]

        n1 = None
        n2 = None

        if robot_nearest_point == (path_len - 1):
            return point_index, None

        n1 = self.find_next_target(path, robot_nearest_point)
        
        if n1 == path_len - 1:
            return n1, None

        n2 = self.find_next_target(path, n1)
         
        if self.calc_dist(path[n1], path[n2]) < self.N / 2.0:
            n1, n2 = n2, None

        return n1, n2

    def find_next_target(self, path, start_index):
        dist = 0

        path_len = path.shape[0]
        cur_index = start_index

        if cur_index == (cur_index - 1):
            return start_index

        cur_index += 1

        while cur_index < path_len and dist < self.N:
            dist += self.calc_dist(path[cur_index], path[cur_index - 1])
            cur_index += 1

        return cur_index - 1

    def calc_dist(self, p1, p2):
        return np.linalg.norm(p1 - p2)

    def create_sub_points(self, path, n):
        u"""
        Args:
            path (np.ndarray).
            n (int).

        Return:
            list.
        """
        points = list()
        first_point = None
        next_point = None
        target_point = path[n]

        if (path.shape[0] - 1) == n:
            first_point = path[n - 1]
            next_point = path[n]
        else:
            first_point = path[n]
            next_point = path[n + 1] 
        
        yaw = np.arctan2(next_point[1] - first_point[1], next_point[0] - first_point[0])

        target_pos = np.zeros((3,))
        target_pos[:2] = target_point
        target_pos[2] = yaw

        points.append(target_pos)

        for i in range(self.SUBPOINTS):
            dp1 = [0, self.SUBPOINTS_DIST * (i+1)]
            dp2 = [0, -self.SUBPOINTS_DIST * (i+1)]

            R = generate_rotation_matrix(yaw)

            p1 = target_point + np.dot(R, dp1)
            p2 = target_point + np.dot(R, dp2)

            pos1 = np.zeros((3,))
            pos1[:2] = p1
            pos1[2] = yaw

            pos2 = np.zeros((3,))
            pos2[:2] = p2
            pos2[2] = yaw

            points.extend([pos1, pos2])

        return points

    def find_yaw_in_point(self, path, n):
        pass


def main():
    import random
    path = np.array([[float(i), 0.0] for i in range(-1, 10)])
    rob_pos = np.array([1.5, 1.1, -0.50])
    LocalTrajectoryGenerator().find_loc_trajectory(path, rob_pos)


if __name__ == '__main__':
    main()
    # generate_trajectories(np.array([0., 0., np.deg2rad(10)]), np.array([10., 0., np.deg2rad(-10)]), 1.5, 4)
