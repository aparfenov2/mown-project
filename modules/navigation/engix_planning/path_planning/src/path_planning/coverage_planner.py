#!/usr/bin/env python3


#!/usr/bin/env python3
from time import sleep
from numpy.lib.function_base import append
import rospy
import numpy as np
from tf.transformations import euler_from_quaternion

from .astart_wrapper import AstarWrapper
from abstractnode import AbstractNode
from continious_state_planner import AstarPathPlanner, GridGenerator, AStar
from coverage_path_planner import CoveragePathClient

from geometry_msgs.msg import Point32, PoseStamped
from nav_msgs.msg import Path, OccupancyGrid, Odometry

from engix_msgs.msg import (Localization, PointWithSpeed, Route, 
                             RouteTaskPolygon, RouteTaskToPoint, ProgressRoutePlanner)


class CoveragePlanner(object):
    IDLE = 'idle'
    COVER = "cover"
    PATH = "to_point"

    COV_STATE_TO_START = 'csts'
    COV_STATE_WALK = 'csw'

    MAX_SPEED = 0.10

    def __init__(self, node, frame):

        self.__astar_planner = AstarWrapper(frame)
        self._frame = frame
        self._node = node

        self.__coverage_path_client = CoveragePathClient()

        self.__state = CoveragePlannerState()

        self.__states_dict = {
            self.__state.IDLE: self.__idle_process,
            self.__state.EXPLORE: self.__polygon_task_progress
        }

        self.obstacles = set()
        self.resolution = 1.0

        self.distance_threshold = 1.0

        self.path_pub = rospy.Publisher(
            '/planner/path_test', Path, queue_size=10)

    def reset(self):
        self.__state.status = self.__state.IDLE

    def initialize(self):
        self.__state.status = self.__state.PROCESS

    def execute(self):
        if self.__state.status != self.__state.IDLE:
            self.__polygon_task_progress()

    def __point32array2route(self, path):
        """
        Args:
            path (Point32[]).

        Return:
            Route.
        """
        route = Route()
        route.header.stamp = rospy.get_rostime()
        route.route = [PointWithSpeed(
            x=point.x, y=point.y, speed=self.MAX_SPEED) for point in path]
        return route

    def __idle_process(self):
        pass

    def __calc_astar_path_to_target(self, target):

        robot_pose = self.__localization2tuple()

        path = self.__astar_planner.search(
            robot_pose, target
        )
        return path

    def __pose2tuple(self, pose):
        q = pose.orientation
        pose = pose.position
        yaw = euler_from_quaternion(np.array([
            q.x, q.y, q.z, q.w
        ]))[2]
        pos_numpy = (pose.x, pose.y, yaw)
        return pos_numpy

    def __localization2tuple(self):
        pos = self._frame.get_localization()
        pos = (pos.pose.x, pos.pose.y, pos.yaw)
        return pos

    def __distance(self, p1, p2):
        p1 = np.array(p1)
        p2 = np.array(p2)
        return np.linalg.norm(p1 - p2)

    def __position2point32(self):
        pos = self._frame.get_localization()
        pos32 = Point32()
        pos32.x = pos.pose.x
        pos32.y = pos.pose.y
        return pos32

    def __polygon_task_progress(self):
        if self._frame.get_localization() is None:
            rospy.logwarn("[RoutePlannerNode] Localization is NONE.")
            return

        if self.__state.status == self.__state.PROCESS:
            self.__state.to_point_path = None
            self.__state.explore_path = None

            source = self.__position2point32()

            task = self._frame.get_coverage_task()
            path = self.__coverage_path_client.get_path(
                task.target_polygon, source
            ).path

            self.__state.explore_path = self.expand_path(path)

            if self.__check_pos_and_first_point_len(self.__state.explore_path):
                # self.__path = self.__add_path_to_start(self.__path)
                self.__state.status = self.__state.GO_TO_POINT

                target_point = (
                    self.__state.explore_path[0].x, self.__state.explore_path[0].y)

                self.__state.to_point_path = self.__calc_astar_path_to_target(
                    target_point)

            else:
                self.__state.status = self.__state.EXPLORE

        elif self.__state.status == self.__state.WAIT_END:
            if self.__check_pos_and_last_point_dist(self.__state.to_point_path):
                self.__state.status = self.__state.EXPLORE

        elif self.__state.status == self.__state.GO_TO_POINT:
            # message = self.__point32array2route(self.__state.to_point_path)

            route = Route()

            for point in self.__state.to_point_path:
                pws = PointWithSpeed()
                pws.x = point[0]
                pws.y = point[1]
                pws.speed = self.MAX_SPEED
                route.route.append(pws)

            self._node.publish_route(route)
            # self.__publish_route(route)

            self.publish_path(self.__state.to_point_path)
            self.__state.status = self.__state.WAIT_END

        elif self.__state.status == self.__state.EXPLORE:
            message = self.__point32array2route(self.__state.explore_path)
            self._node.publish_route(message)
            # self.__publish_route(message)

            self.publish_path(self.__state.explore_path)
            self.__state.status = self.__state.WAIT_EXPLORE

        elif self.__state.status == self.__state.WAIT_EXPLORE:
            if self.__check_pos_and_last_point_dist(self.__state.explore_path):
                self.__state.status = self.__state.IDLE

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

    def publish_path(self, points):
        # path_pub = rospy.Publisher('/planner/path_test', Path, queue_size=10)
        path = Path()
        path.header.stamp = rospy.get_rostime()
        path.header.frame_id = 'world'
        for point in points:
            pose = PoseStamped()
            pose.header = path.header
            if isinstance(point, list):
                pose.pose.position.x = point[0]
                pose.pose.position.y = point[1]
            else:
                pose.pose.position.x = point.x
                pose.pose.position.y = point.y
            path.poses.append(pose)
        self.path_pub.publish(path)
        # sleep(1.0)
        # self.path_pub.publish(path)

    def __check_pos_and_first_point_len(self, path):
        u"""
        Args:
            path (Point32[]).

        Return:
            bool.
        """
        if len(path) < 2:
            return True

        ego_pos = self._frame.get_localization()

        first_point = np.array((path[0].x, path[0].y))
        current_pos = np.array(
            (ego_pos.pose.x, ego_pos.pose.y)
        )

        return np.linalg.norm(first_point - current_pos) > self.distance_threshold

    def __check_pos_and_last_point_dist(self, path):
        u"""
        Args:
            path (Point32[]).

        Return:
            bool.
        """
        if len(path) < 2:
            return True

        if isinstance(path[-1], list):
            first_point = np.array((path[-1][0], path[-1][1]))
        else:
            first_point = np.array((path[-1].x, path[-1].y))

        ego_pos = self._frame.get_localization()
        current_pos = np.array(
            (ego_pos.pose.x, ego_pos.pose.y)
        )

        return np.linalg.norm(first_point - current_pos) < 0.5

    def __add_path_to_start(self, path):
        target = (path[0].x, path[0].y)

        path_to_target = self.__calc_astar_path_to_target(target)

        points_to_start = list()
        for p in path_to_target:
            points_to_start.append(Point32(
                x=p[0],
                y=p[1]
            ))

        path = points_to_start + path
        return path


class CoveragePlannerState(object):
    IDLE = 'idle'
    PROCESS = 'process'
    GO_TO_POINT = 'go_to_point'
    WAIT_END = 'wait_end'
    EXPLORE = 'explore'
    WAIT_EXPLORE = 'wait_explore'

    def __init__(self):
        self.status = self.IDLE
        self.to_point_path = None
        self.explore_path = None
