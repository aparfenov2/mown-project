#!/usr/bin/env python3
from time import sleep

from numpy.lib.function_base import append
import rospy
import numpy as np
from task_behavior_engine.tree import Node, NodeStatus
from tf.transformations import euler_from_quaternion

from .sequence_decorator import SequenceDecorator
from behavior_planning.algorithms import AstarWrapper
from behavior_planning.algorithms import ContinuosAstarWrapper

from geometry_msgs.msg import Point32, PoseStamped
from nav_msgs.msg import Path, OccupancyGrid
from enginx_msgs.msg import (Localization, PointWithSpeed, Route,
                             RouteTaskPolygon, RouteTaskToPoint, ProgressRoutePlanner)


class TrajectoryPublisher(SequenceDecorator):
    def __init__(self, name, frame, *args, **kwargs):
        super(TrajectoryPublisher, self).__init__(name=name,
                                                  *args, **kwargs)
        self._frame = frame
        self.__astar_planner = ContinuosAstarWrapper(frame)

        self._stop_planning_threshold = 0.5

        self.distance_threshold = 1.0

        # self.__route_publisher = rospy.Publisher(
        #     rospy.get_param('/planner/topics/route/path_planner'),
        #     Route,
        #     queue_size=10
        # )
        # self.__progress_route_publisher = \
        #     rospy.Publisher(rospy.get_param('/planner/topics/path_progress'),
        #                     ProgressRoutePlanner,
        #                     queue_size=10)

    def node_tick(self, nodedata):
        if self._frame.get_localization() is None:
            return

        target_pose = self._frame.get_path_task().target_pose
        target_pose = self.__pose2tuple(target_pose)

        loca = self._frame.get_localization()
        robot_pose = self.__localization2tuple(loca)

        if self.__distance(target_pose, robot_pose) < self._stop_planning_threshold:
            return

        path = self.__calc_astar_path_to_target(target_pose)

        route = Route()

        for point in path:
            pws = PointWithSpeed()
            pws.x = point[0]
            pws.y = point[1]
            pws.speed = self.MAX_SPEED
            route.route.append(pws)

        self._frame.set_trajectory(route)
        # self._node.publish_route(route)

        # self.__send_task_done()
        return NodeStatus.SUCCESS

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


class State(object):
    def __init__(self, frame, parent) -> None:
        self._frame = frame
        self._parent = parent

    def tick(self, nodedata):
        raise NotImplementedError()


class ProcessState(State):
    def tick(self, nodedata):
        state = self._parent.state
        state.to_point_path = None
        state.explore_path = None

        source = self.__position2point32()

        task = self._frame.get_coverage_task()
        path = self._parent.coverage_path_client.get_path(
            task.target_polygon, source
        ).path

        state.explore_path = self.expand_path(path)

        if self.__check_pos_and_first_point_len(self.__state.explore_path):
            # self.__path = self.__add_path_to_start(self.__path)
            state.status = state.GO_TO_POINT

            target_point = (
                state.explore_path[0].x, state.explore_path[0].y)

            state.to_point_path = self.__calc_astar_path_to_target(
                target_point)

        else:
            state.status = state.EXPLORE

    def __position2point32(self):
        pos = self._frame.get_localization()
        pos32 = Point32()
        pos32.x = pos.pose.x
        pos32.y = pos.pose.y
        return pos32

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

    def __calc_astar_path_to_target(self, target):

        robot_pose = self.__localization2tuple()

        path = self.__astar_planner.search(
            robot_pose, target
        )
        return path


class WaitEndState(State):
    def tick(self, nodedata):
        state = self._parent.state
        if self.__check_pos_and_last_point_dist(state.to_point_path):
            state.status = state.EXPLORE
    
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


class GoToPointState(State):
    def tick(self, nodedata):
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


class ExploreState(State):
    def tick(self, nodedata):
        message = self.__point32array2route(self.__state.explore_path)
        self._node.publish_route(message)
        # self.__publish_route(message)

        self.publish_path(self.__state.explore_path)
        self.__state.status = self.__state.WAIT_EXPLORE


class WaitExploreState(State):
    def tick(self, nodedata):
        if self.__check_pos_and_last_point_dist(self.__state.explore_path):
            self.__state.status = self.__state.IDLE


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