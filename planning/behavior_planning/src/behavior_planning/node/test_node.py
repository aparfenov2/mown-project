from json import load
import time
import math

import dubins
import numpy as np
import rospy
from task_behavior_engine.tree import Node, NodeStatus
from .speed_generator import SpeedGenerator

from enginx_msgs.msg import Route, PointWithSpeed


class TestTrajectoryNode(Node):
    def __init__(self, name, frame, *args, **kwargs):
        super(TestTrajectoryNode, self).__init__(name=name,
                                                 run_cb=self.run,
                                                 *args, **kwargs)
        self._frame = frame
        self._published = False

    def run(self, nodedata):
        if (not self._frame.has_localization()):
            return NodeStatus(NodeStatus.FAIL, "Frame doesn't have a localization")

        if not self._published:
            turning_radius = 3.0
            step_size = 0.1

            localization = self._frame.get_localization()
            x_0 = (localization.pose.x, localization.pose.y, localization.yaw)
            x_f = (localization.pose.x + 6, localization.pose.y + 2, np.deg2rad(100))

            path = dubins.shortest_path(x_0, x_f, turning_radius)
            configurations, _ = path.sample_many(step_size)

            route = Route()
            route.header.stamp = self._frame.get_localization().header.stamp
            for x, y, yaw in configurations:
                pws = PointWithSpeed()
                pws.x = x
                pws.y = y
                pws.speed = 0.8
                pws.d_time = 0.0
                route.route.append(pws)
            self._frame.set_trajectory(route)

            for i in range(1, 20):
                route.route[-i].speed = min(i * 0.01, route.route[i].speed)

            self._published = True
            print(f"Speed: {[r.speed for r in route.route]}")
        else:
            pass

        return NodeStatus(NodeStatus.SUCCESS)


class TestStraightLineNode(Node):
    def __init__(self, name, frame, line_len, *args, **kwargs):
        super(TestStraightLineNode, self).__init__(name=name,
                                                   run_cb=self.run,
                                                   *args, **kwargs)
        self._frame = frame
        self._published = False
        self._line_len = line_len
        self._speed_generator = SpeedGenerator(0.5, 5, 5)

    def run(self, nodedata):
        if (not self._frame.has_localization()):
            return NodeStatus(NodeStatus.FAIL, "Frame doesn't have a localization")

        if not self._published:
            time.sleep(0.5)
            turning_radius = 3.0
            step_size = 0.1

            localization = self._frame.get_localization()
            x_0 = (localization.pose.x, localization.pose.y, localization.yaw)
            x_f = (localization.pose.x + self._line_len * np.cos(localization.yaw),
                   localization.pose.y + self._line_len * np.sin(localization.yaw),
                   localization.yaw)

            path = dubins.shortest_path(x_0, x_f, turning_radius)
            configurations, _ = path.sample_many(step_size)

            route = Route()
            route.header.stamp = self._frame.get_localization().header.stamp
            for x, y, yaw in configurations:
                pws = PointWithSpeed()
                pws.x = x
                pws.y = y
                # pws.speed = 0.8
                pws.d_time = 0.0
                route.route.append(pws)

            self._speed_generator.fill_speeds(route, True)

            self._frame.set_trajectory(route)

            self._published = True
            print(f"Speed: {[r.speed for r in route.route]}")
        else:
            pass

        return NodeStatus(NodeStatus.SUCCESS)


class TestCirclesNode(Node):
    def __init__(self, name, frame, target_speed, left_distance=None, right_distance=None, *args, **kwargs):
        super(TestCirclesNode, self).__init__(name=name,
                                              run_cb=self.run,
                                              *args, **kwargs)
        if left_distance is None and right_distance is None:
            raise Exception("left_distance or/and right_distance need to be set.")

        self._frame = frame
        self._computed = False
        self._left_distance = left_distance
        self._right_distance = right_distance
        self._target_speed = target_speed
        self._speed_generator = SpeedGenerator(target_speed, 5, 5)
        self._paths = []
        self._current = -1

    def run(self, nodedata):
        if (not self._frame.has_localization()):
            return NodeStatus(NodeStatus.FAIL, "Frame doesn't have a localization")

        if not self._computed:
            time.sleep(0.5)
            print("[Path] Start compute path.")

            localization = self._frame.get_localization()
            localization = (localization.pose.x, localization.pose.y, localization.yaw)

            if self._left_distance is not None:
                route1 = Route()
                route2 = Route()

                orientation_1 = localization[2] - np.pi / 2.0
                first_point = (localization[0] + self._left_distance * np.cos(orientation_1),
                               localization[1] + self._left_distance * np.sin(orientation_1),
                               localization[2] - np.pi)

                self.circle_path_to_point(localization, first_point, self._left_distance / 2.0, route1)

                self._paths.append((first_point, route1))

                self.circle_path_to_point(first_point, localization, self._left_distance / 2.0, route2)

                self._paths.append((localization, route2))

            if self._right_distance is not None:
                route1 = Route()
                route2 = Route()

                orientation_1 = localization[2] + np.pi / 2.0
                first_point = (localization[0] + self._right_distance * np.cos(orientation_1),
                               localization[1] + self._right_distance * np.sin(orientation_1),
                               localization[2] - np.pi)

                self.circle_path_to_point(localization, first_point, self._right_distance / 2.0, route1)

                self._paths.append((first_point, route1))

                self.circle_path_to_point(first_point, localization, self._right_distance / 2.0, route2)

                self._paths.append((localization, route2))

            # orientation_2 = localization.yaw + np.pi / 2.0
            # first_point = (localization.pose.x + self._left_distance * np.cos(orientation_2),
            #                localization.pose.y + self._left_distance * np.sin(orientation_2),
            #                localization.yaw + np.pi)

            # self.circle_path_to_point(first_point, self._left_distance / 2.0, route)

            # self._speed_generator.fill_speeds(route, True)

            # self._frame.set_trajectory(route)

            self._computed = True
        else:
            pass

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
        localization = self._frame.get_localization()
        localization = (localization.pose.x, localization.pose.y, localization.yaw)
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
