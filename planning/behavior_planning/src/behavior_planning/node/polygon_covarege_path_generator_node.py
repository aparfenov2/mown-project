from cProfile import label
from math import fabs
from select import select
from time import time

import casadi
import rospy
import numpy as np
from scipy.optimize import NonlinearConstraint
from scipy import optimize
from task_behavior_engine.tree import Node, NodeStatus

from coverage_path_planner import CoveragePathClient

from geometry_msgs.msg import Point, Point32
from enginx_msgs.msg import Route, PointWithSpeed, SpeedGeneratorDebug


class CovaragePathGeneratorNode(Node):
    def __init__(self, name, frame, *args, **kwargs):
        super(CovaragePathGeneratorNode, self).__init__(name=name,
                                                 run_cb=self.run,
                                                 *args, **kwargs)
        self._frame = frame
        self.__coverage_path_client = CoveragePathClient()

    def run(self, nodedata):
        rospy.logerr(f"[CovaragePathGeneratorNode] Enter")

        source = self.__position2point32()

        task = self._frame.route_task_task_polygon_message
        path = self.__coverage_path_client.get_path(
            task.target_polygon, source
        ).path

        expanded_path = self.expand_path(path)

        rospy.logerr(f"[CovaragePathGeneratorNode] expanded_path {len(expanded_path)}")

        route = Route()
        route.header.stamp = self._frame.get_localization().header.stamp
        for point in expanded_path:
            pws = PointWithSpeed()
            pws.x = point.x
            pws.y = point.y
            pws.speed = 0.0
            pws.d_time = 0.0
            route.route.append(pws)
        self._frame.set_trajectory(route)

        return NodeStatus(NodeStatus.SUCCESS)

    def expand_path(self, path):
        expanded_path = list()

        for i in range(1, len(path)):
            p1 = path[i - 1]
            p2 = path[i]

            dist = self.calc_distance_btwn_points(p1, p2)
            if dist > 1.0:
                parts = int(dist / 0.2)
                new_points = self.get_equidistant_points(
                    (p1.x, p1.y),
                    (p2.x, p2.y),
                    parts
                )
                converted_points = self.convert_list_to_point3_array(
                    new_points)
                expanded_path += converted_points
            else:
                expanded_path += [p1, p2]

        return expanded_path

    def get_equidistant_points(self, p1, p2, parts):
        return list(zip(np.linspace(p1[0], p2[0], parts+1),
                        np.linspace(p1[1], p2[1], parts+1)))

    def calc_distance_btwn_points(self, p1, p2):
        v1 = np.array([p1.x, p1.y])
        v2 = np.array([p2.x, p2.y])
        return np.linalg.norm(v1 - v2)

    def convert_list_to_point3_array(self, path):
        new_path = list()

        for point in path:
            p = Point32(
                x=point[0],
                y=point[1]
            )
            new_path.append(p)
        return new_path

    def __position2point32(self):
        pos = self._frame.get_localization()
        pos32 = Point32()
        pos32.x = pos.pose.x
        pos32.y = pos.pose.y
        return pos32
