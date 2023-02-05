from json import load
import time
import math

import dubins
import numpy as np
import rospy
from task_behavior_engine.tree import Node, NodeStatus
from .speed_generator import SpeedGenerator

from enginx_msgs.msg import Route, PointWithSpeed


class CircleMovingNode(Node):
    def __init__(self, name, frame, *args, **kwargs):
        super(CircleMovingNode, self).__init__(name=name,
                                               run_cb=self.run,
                                               *args, **kwargs)
        self._frame = frame
        self._paths = []
        self._current = -1
        self._last_stamp = 0.0
        self._speed_generator = None

    def run(self, nodedata):

        if (not self._frame.localization.has_localization()):
            rospy.loginfo_throttle(1, "Waiting for localization.")
            return NodeStatus(NodeStatus.FAIL, "Frame doesn't have a localization")

        if (not self._frame.circle_moving_task.has_message()):
            rospy.loginfo_throttle(1, "Waiting for LineMovingTask message.")
            return NodeStatus(NodeStatus.FAIL, "Frame doesn't have a LineMovingTask message")

        cmt_stamp_time = self._frame.circle_moving_task.stamp_sec
        ptt_stamp_time = self._frame.planning_task_type.stamp_sec

        if (self._last_stamp < cmt_stamp_time
                and cmt_stamp_time >= ptt_stamp_time):
            self._paths = list()
            left_radius = self._frame.circle_moving_task.left_radius
            right_radius = self._frame.circle_moving_task.right_radius
            self._target_speed = self._frame.circle_moving_task.target_speed
            self._speed_generator = SpeedGenerator(self._target_speed, 5, 5)

            localization = self._frame.localization.pose

            if left_radius > 0.0:
                route1 = Route()
                route2 = Route()

                orientation_1 = localization[2] - np.pi / 2.0
                first_point = (localization[0] + 2.0 * left_radius * np.cos(orientation_1),
                               localization[1] + 2.0 * left_radius * np.sin(orientation_1),
                               localization[2] - np.pi)

                self.circle_path_to_point(localization, first_point, left_radius, route1)

                self._paths.append((first_point, route1))

                self.circle_path_to_point(first_point, localization, left_radius, route2)

                self._paths.append((localization, route2))

            if right_radius > 0.0:
                route1 = Route()
                route2 = Route()

                orientation_1 = localization[2] + np.pi / 2.0
                first_point = (localization[0] + 2.0 * right_radius * np.cos(orientation_1),
                               localization[1] + 2.0 * right_radius * np.sin(orientation_1),
                               localization[2] - np.pi)

                self.circle_path_to_point(localization, first_point, right_radius, route1)

                self._paths.append((first_point, route1))

                self.circle_path_to_point(first_point, localization, right_radius, route2)

                self._paths.append((localization, route2))

            self._current == -1
            self._last_stamp = self._frame.circle_moving_task.stamp_sec

        if self._current == -1:
            self._current = 0
            temp_route = self._paths[self._current][1]
            self._speed_generator.fill_speeds(temp_route)
            self._frame.set_trajectory(temp_route)
        else:
            if self.calculate_distance(self._paths[self._current][0]) < 0.5:
                self._current = (self._current + 1) % len(self._paths)
                temp_route = self._paths[self._current][1]
                self.fill_target_speed(temp_route)
                self._frame.set_trajectory(temp_route)
            else:
                temp_route = self._paths[self._current][1]
                self._frame.set_trajectory(temp_route)

        return NodeStatus(NodeStatus.SUCCESS)

    def calculate_distance(self, target_point):
        localization = self._frame.localization.pose
        return math.dist(localization[:2], target_point[:2])

    def fill_target_speed(self, route):
        for i in range(0, len(route.route)):
            route.route[i].speed = self._target_speed

    def circle_path_to_point(self, start_point, target_point, radius, out_route):
        turning_radius = radius
        step_size = 0.1

        x_0 = start_point
        x_f = target_point

        path = dubins.shortest_path(x_0, x_f, turning_radius)
        configurations, _ = path.sample_many(step_size)

        for x, y, yaw in configurations:
            pws = PointWithSpeed()
            pws.x = x
            pws.y = y
            # pws.speed = 0.8
            pws.d_time = 0.0
            out_route.route.append(pws)
