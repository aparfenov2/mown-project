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

from geometry_msgs.msg import Point
from geometry_msgs.msg import Point32, PoseStamped
from nav_msgs.msg import Path, OccupancyGrid
from enginx_msgs.msg import (Localization, PointWithSpeed, Route,
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
        trajectory = self._smoother.smooth(
            points, robot_pose, target
        )
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


class TrajectorySmoother:
    def __init__(self, w: list) -> None:
        self._w1 = w[0]
        self._w2 = w[1]
        self._w3 = w[2]
        self._w4 = w[3]

    def smooth(self, points, start_point, target_point):
        opti = casadi.Opti()

        points_count = len(points)

        result = {
            'x': [0.0] * points_count,
            'y': [0.0] * points_count
        }

        x = opti.variable()
        y = opti.variable()

        cost = 0.0
        # opti.set_initial(s, s_init)
        # opti.set_initial(ds, ds_init)
        # opti.set_initial(dds, dds_init)

        opti.subject_to(x == start_point[0])
        opti.subject_to(y == start_point[1])
        # opti.minimize(
        #     + self._w3 * (ds - ds_init)**2
        #     + self._w4 * (dds - dds_init)**2)
        # opti.subject_to(opti.bounded(s_min, s, s_max))
        # opti.subject_to(opti.bounded(ds_min, ds, ds_max))
        # opti.subject_to(opti.bounded(dds_min, dds, dds_max))

        prev_x = x
        prev_y = y

        result['x'][0] = x
        result['y'][0] = y

        for i in range(1, points_count - 1):
            x_i = opti.variable()
            y_i = opti.variable()

            cost += (
                self._w1 * (x_i - prev_x)**2
                + self._w2 * (y_i - prev_y)**2
                + self._w3 * (x_i - points[i][0])**2
                + self._w4 * (y_i - points[i][1])**2)
            # opti.minimize(
            #     self._w1 * (s_target - s_i)**2
            #     + self._w2 * (casadi.if_else(s >= s_target - 0.01, ds, 0.0))**2
            #     + self._w2 * (casadi.if_else(ds <= ds_target, ds_target - ds, (ds)))**2
            #     + self._w3 * (ds_i - prev_ds)**2
            #     + self._w4 * (dds_i - prev_dds)**2)
            # opti.subject_to(opti.bounded(s_min, s_i, s_max))
            # opti.subject_to(opti.bounded(ds_min, ds_i, ds_max))
            # opti.subject_to(opti.bounded(dds_min, dds_i, dds_max))
            # opti.subject_to(opti.bounded(ddds_min, (dds_i - prev_ds) / self._dt, ddds_max))
            # opti.subject_to(prev_s + ds_i * self._dt + dds_i * 0.5 * self._dt ** 2 == s_i)
            # opti.sutrajectorybject_to(prev_ds + dds_i * self._dt == ds_i)
            # opti.subject_to(
            #     ds_i == casadi.if_else(s_i >= s_target * 0.8,
            #                            opti.bounded(ds_min, ds_i, 0.5),
            #                            opti.bounded(ds_min, ds_i, ds_max)))
            # opti.subject_to(opti.bounded(ds_min, ds_i, casadi.if_else(s_i >= s_target * 0.8, ds_max * 0.5, ds_max)))

            # opti.subject_to(ds_i == casadi.if_else(s_i >= s_target, 0.0, ds_max))
            # opti.subject_to(ds_i >= casadi.if_else(s_i >= s_target, 0.0, ds_min))
            # opti.subject_to(ds_i <= casadi.if_else(s_i >= s_target, 0.0, ds_max))
            # opti.subject_to(opti.bounded(casadi.if_else(s_i >= s_target, 0.0, ds_min),
            #                              ds_i,
            #                              casadi.if_else(s_i >= s_target, 0.0, ds_max)))

            prev_x = x_i
            prev_y = y_i

            result['x'][i] = prev_x
            result['y'][i] = prev_y

        x = opti.variable()
        y = opti.variable()

        opti.subject_to(x == target_point[0])
        opti.subject_to(y == target_point[1])

        result['x'][-1] = x
        result['y'][-1] = y

        opti.minimize(cost)
        opts = {'ipopt.print_level': 0, 'print_time': 0, 'ipopt.sb': 'yes'}
        opti.solver('ipopt', opts)

        sol = opti.solve()

        trajectory = {
            'x': [0.0] * points_count,
            'y': [0.0] * points_count
        }

        for i in range(points_count):
            trajectory['x'][i] = sol.value(result['x'][i])
            trajectory['y'][i] = sol.value(result['y'][i])

        return trajectory


if __name__ == "__main__":
    smoother = TrajectorySmoother([10.0, 10.0, 10.1, 10.1])
    points = [[0.5, 0.5], [1.5, 1.5], [2.5, 2.5], [3.5, 3.5], [4.5, 4.5], [5.5, 5.5]] 
    start_point = [0.25, 0.33]
    target_point = [5.6, 5.2]

    trajectory = smoother.smooth(
        points, start_point, target_point
    )

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