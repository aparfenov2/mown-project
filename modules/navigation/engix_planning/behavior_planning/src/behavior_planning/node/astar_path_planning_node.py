#!/usr/bin/env python3
from time import sleep

import casadi
from numpy.lib.function_base import append
import rospy
import numpy as np
from task_behavior_engine.tree import Node, NodeStatus
from tf.transformations import euler_from_quaternion

from behavior_planning.algorithms import AstarWrapper
from behavior_planning.algorithms import ContinuosAstarWrapper
from .trajectory_smoother import TrajectorySmoother

from geometry_msgs.msg import Point
from geometry_msgs.msg import Point32, PoseStamped
from nav_msgs.msg import Path, OccupancyGrid
from engix_msgs.msg import (Localization, PointWithSpeed, Route,
                             RouteTaskPolygon, RouteTaskToPoint, ProgressRoutePlanner)


class AstarPathPlanningNode(Node):

    MAX_SPEED = 2.0

    def __init__(self, name, frame, *args, **kwargs):
        super(AstarPathPlanningNode, self).__init__(name=name,
                                                    run_cb=self.run,
                                                    *args, **kwargs)
        self._frame = frame
        self.__astar_planner = ContinuosAstarWrapper(frame)
        self._smoother = TrajectorySmoother([10.0, 10.0, 10.1, 10.1])

        self._stop_planning_threshold = 0.5

        self.distance_threshold = 1.0

    def run(self, nodedata):
        if self._frame.get_path_done():
            return NodeStatus(NodeStatus.SUCCESS)

        if self._frame.get_localization() is None:
            return NodeStatus(NodeStatus.FAIL, f"Node {self.name}, NO localization.")

        target_pose = self._frame.route_task_to_point_message.target_pose
        target_pose = self.__pose2tuple(target_pose)

        loca = self._frame.get_localization()
        robot_pose = self.__localization2tuple(loca)

        if self.__distance(target_pose, robot_pose) < self._stop_planning_threshold:
            return NodeStatus(NodeStatus.SUCCESS)

        if not self._frame.has_astar_path() or self._need_to_replan(robot_pose):
            path_ = self.__calc_astar_path_to_target(target_pose)
            self._frame.set_astar_path(path_)
        else:
            path_ = self._frame.astar_path
        self.__fill_debug_trajectory(path_, self._frame.planning_debug.astart_trajectory)
        path = self.__smooth_trajectory(path_, target_pose)
        self.__fill_debug_trajectory(path, self._frame.planning_debug.smoothed_trajectory)

        self._frame.discrete_trajectory.set_points(path)

        # route = Route()

        # for point in path:
        #     pws = PointWithSpeed()
        #     pws.x = point[0]
        #     pws.y = point[1]
        #     pws.speed = self.MAX_SPEED
        #     route.route.append(pws)

        # self._frame.set_trajectory(route)
        self._frame.set_path_done(True)
        return NodeStatus(NodeStatus.SUCCESS)

    def _need_to_replan(self, robot_position):
        path_rtree = self._frame.astar_path_rtree

        if path_rtree is None:
            return True

        nearest_point_index = list(self._s_indexes.nearest((robot_position[0], robot_position[1]), 1))[0]
        nearest_point = self._frame.astar_path[nearest_point_index]
        dist = self.__distance(robot_position[:2], nearest_point)

        if dist > 2.0:
            return True

        return False

    def __fill_debug_trajectory(self, points, debug_list):
        for x, y in points:
            point = Point(x=x, y=y)
            debug_list.append(point)

    def __smooth_trajectory(self, points, target):
        loca = self._frame.get_localization()
        robot_pose = self.__localization2tuple(loca)
        # points[0] = robot_pose[:2]
        points[-1] = target[:2]
        trajectory = self._smoother.smooth(points, robot_pose)
        new_points = list()
        for i in range(len(trajectory['x'])):
            new_points.append([trajectory['x'][i], trajectory['y'][i]])

        # rospy.logwarn(f"{points[:10]=}\n{new_points[:10]=}")
        return new_points

    def __calc_astar_path_to_target(self, target):
        loca = self._frame.get_localization()
        robot_pose = self.__localization2tuple(loca)

        path = self.__astar_planner.search(robot_pose, target)
        return path

    def __pose2tuple(self, pose):
        q = pose.orientation
        pose = pose.position
        yaw = euler_from_quaternion(np.array([
            q.x, q.y, q.z, q.w
        ]))[2]
        pos_numpy = (pose.x, pose.y, yaw)
        return pos_numpy

    def __localization2tuple(self, pos):
        pos = (pos.pose.x, pos.pose.y, pos.yaw)
        return pos

    def __distance(self, p1, p2):
        p1 = np.array(p1)
        p2 = np.array(p2)
        return np.linalg.norm(p1 - p2)


class TrajectorySmootherOld:
    def __init__(self, w: list, raise_exception=True) -> None:
        self._w1 = w[0]
        self._w2 = w[1]
        self._w3 = w[2]
        self._w4 = w[3]

        self._raise_exception = raise_exception

    def smooth(self, path, robot_pose):
        opti = casadi.Opti()

        W0 = 0.1
        W1 = 100.0
        W2 = 10.0
        # W3 = 1.0

        EPS = 10e-6
        K_MAX = 2.0

        KNOTS = len(path)
        X = opti.variable(KNOTS)
        Y = opti.variable(KNOTS)
        DX = opti.variable(KNOTS - 1)
        DY = opti.variable(KNOTS - 1)
        # DPHI = opti.variable(KNOTS - 1)
        # THETA = opti.variable()

        cost = 0.0

        opti.subject_to(X[0] == robot_pose[0])
        opti.subject_to(Y[0] == robot_pose[1])
        # opti.subject_to(THETA == robot_pose[2])
        # opti.subject_to(THETA == casadi.atan2(DY[0], DX[0] + 10e-6))

        for i in range(1, KNOTS - 1):
            opti.set_initial(X[i], path[i][0])
            opti.set_initial(Y[i], path[i][1])
            opti.subject_to(X[i] == X[i - 1] + DX[i - 1])
            opti.subject_to(Y[i] == Y[i - 1] + DY[i - 1])
            # opti.subject_to(DPHI[i - 1] == casadi.atan2(DY[i], DX[i] + EPS) - casadi.atan2(DY[i - 1], DX[i - 1] + EPS))

            cost0 = W0 * ((X[i] - path[i][0]) ** 2 + (Y[i] - path[i][1]) ** 2)
            # cost1 = W1 * ((DX[i] - DX[i - 1]) ** 2 + (DY[i] - DY[i - 1]) ** 2)
            # cost2 = W2 * (DPHI[i] - DPHI[i - 1]) ** 2
            cost3 = W1 * ((X[i] - (X[i-1] + X[i+1]) / 2.0) ** 2 + (Y[i] - (Y[i-1] + Y[i+1]) / 2.0) ** 2)
            # cost2 = W2 * ((DPHI[i - 1]) / (casadi.sqrt(DX[i - 1] ** 2 + DY[i - 1] ** 2) + EPS) - K_MAX) ** 2

            # cost += cost0 + cost1 + cost2
            cost += cost3 + cost0

        last_index = len(path) - 1
        delta_last_index = KNOTS - 2

        opti.subject_to(X[last_index] == path[-1][0])
        opti.subject_to(Y[last_index] == path[-1][1])
        opti.subject_to(X[last_index] == X[last_index - 1] + DX[delta_last_index])
        opti.subject_to(Y[last_index] == Y[last_index - 1] + DY[delta_last_index])
        # cost += W2 * ((DPHI[last_index - 1]) / (casadi.sqrt(DX[last_index - 1] ** 2 + DY[last_index - 1] ** 2)) - K_MAX) ** 2

        opti.minimize(cost)
        opts = {'ipopt.print_level': 0, 'print_time': 0, 'ipopt.sb': 'yes'}
        opti.solver('ipopt', opts)

        # times = [0.0] * KNOTS

        try:
            sol = opti.solve()

            trajectory = {
                'x': [0.0] * KNOTS,
                'y': [0.0] * KNOTS
            }

            for i in range(KNOTS):
                trajectory['x'][i] = sol.value(X[i])
                trajectory['y'][i] = sol.value(Y[i])

            return trajectory
        except Exception as e:
            if self._raise_exception:
                save_data = {
                    "robot_position": robot_pose,
                    "path": path
                }

                import json
                with open('/tmp/json_data.json', 'w') as outfile:
                    json.dump(save_data, outfile)

                raise Exception()
            else: 
                trajectory = {
                    'x': [0.0] * KNOTS,
                    'y': [0.0] * KNOTS
                }

                for i in range(KNOTS):
                    trajectory['x'][i] = opti.debug.value(X[i])
                    trajectory['y'][i] = opti.debug.value(Y[i])
                
                return trajectory


def test_smoothier_1():
    smoother = TrajectorySmoother([10.0, 10.0, 10.1, 10.1], False)
    points = [[0.5, 0.5], [1.5, 1.8], [2.5, 2.4], [3.5, 3.5], [4.5, 4.5], [5.5, 5.5]] 
    start_point = [0.25, 0.33, np.deg2rad(-10)]
    target_point = [5.6, 5.2]

    trajectory = smoother.smooth(points, start_point)

    import matplotlib.pyplot as plt
    fig = plt.figure()
    ax = plt.axes()
    x, y = [], []
    for x_, y_ in points:
        x.append(x_)
        y.append(y_)

    ax.plot(trajectory['x'], trajectory['y'], label='x')
    ax.plot(x, y, label='dx')

    plt.legend()
    plt.show()


def test_smoothier_from_file():
    import json

    with open('/tmp/json_data.json') as json_file:
        data = json.load(json_file)
        print(data)

    smoother = TrajectorySmoother([10.0, 10.0, 10.1, 1.1])
    points = data['path']
    # start_point = data['robot_position']
    # start_point = [data['path'][0][0], data['path'][0][1], data['robot_position'][2]]
    start_point = [
        data['path'][0][0],
        data['path'][0][1],
        np.arctan2(data['path'][1][1] - data['path'][0][1],
                   data['path'][1][0] - data['path'][0][0])
    ]

    trajectory = smoother.smooth(points, start_point)

    import matplotlib.pyplot as plt
    fig = plt.figure()
    ax = plt.axes()
    x, y = [], []
    for x_, y_ in points:
        x.append(x_)
        y.append(y_)

    # ax.plot(trajectory['x'], trajectory['y'], label='x')
    ax.plot(x, y, label='dx')
    ax.scatter(start_point[0], start_point[1], label='dx')

    plt.legend()
    plt.show()


if __name__ == "__main__":
    test_smoothier_from_file()
