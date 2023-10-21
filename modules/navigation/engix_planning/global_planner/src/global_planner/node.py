#!/usr/bin/env python3

import threading
import rospy

from abstractnode import AbstractNode
from global_planner.common.frame import Frame
from global_planner.planners.straight_line_planner import StraightLinePlanner
from global_planner.planners.dubins_path_planner import DubinsPathPlanner
from global_planner.planners.coverage_path_planner import CoveragePathPlanner

from nav_msgs.msg import Path
from engix_msgs.msg import (
    Localization,
    CoverageTask,
    LineMovingTask,
    DubinsPlanningTask,
)
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, Pose, Quaternion
from std_msgs.msg import ColorRGBA


class GlobalPlanningNode(AbstractNode):
    def initialization(self):
        self._rlock = threading.RLock()
        self._frame = Frame()
        self._straight_line_planner = StraightLinePlanner()
        self._dubins_planning_task_planner = DubinsPathPlanner()
        self._coverage_planner = CoveragePathPlanner()

        self._current_planner_is_coverage = False

        self._path_publisher = rospy.Publisher(
            rospy.get_param("~topics/result_path"), Path, queue_size=10
        )

        self.__visualization_publisher = rospy.Publisher(
            "/global_planner/visualization",
            MarkerArray,
            queue_size=1,
        )

        rospy.Subscriber(
            rospy.get_param("~topics/localization"),
            Localization,
            self.localization_callback,
        )
        rospy.Subscriber(
            rospy.get_param("~topics/straight_line_task"),
            LineMovingTask,
            self.straight_line_task_callback,
        )
        rospy.Subscriber(
            rospy.get_param("~topics/dubins_planning_task"),
            DubinsPlanningTask,
            self.dubins_planning_task_callback,
        )
        rospy.Subscriber(
            rospy.get_param("~topics/coverage_planning_task"),
            CoverageTask,
            self.coverage_task_callback,
        )

    def localization_callback(self, message) -> None:
        with self._rlock:
            self._frame.set_localization(message)

    def straight_line_task_callback(self, message) -> None:
        with self._rlock:
            self._frame.set_line_moving_task(message)
            self._current_planner_is_coverage = False
            path = self._straight_line_planner.plan(self._frame)
            self.send_path(path)

    def dubins_planning_task_callback(self, message) -> None:
        with self._rlock:
            self._frame.set_dubins_planning_task(message)
            self._current_planner_is_coverage = False
            path = self._dubins_planning_task_planner.plan(self._frame)
            self.send_path(path)

    def coverage_task_callback(self, message: CoverageTask) -> None:
        with self._rlock:
            self._frame.set_coverage_task(message)
            self._current_planner_is_coverage = True
            self._coverage_planner.reset()

    def send_path(self, path) -> None:
        path.header.frame_id = "odom"
        path.header.stamp = rospy.get_rostime()

        for pose in path.poses:
            pose.header = path.header

        self._path_publisher.publish(path)

    def work(self) -> None:
        with self._rlock:
            if self._current_planner_is_coverage:
                path = self._coverage_planner.plan(self._frame)
                if path is not None:
                    self.send_path(path)

                self.__visualize_coverage_path()

    def __visualize_coverage_path(self):
        polygon = self._coverage_planner.polygon
        path_to_start = self._coverage_planner.path_to_start
        coverage_plan = self._coverage_planner.coverage_path

        polygon_marker = self.__create_path_marker_from_list(polygon, 1, ColorRGBA(1.0, 0.0, 0.0, 1.0))
        path_to_start_marker = self.__create_path_marker_from_path(path_to_start, 2, ColorRGBA(1.0, 0.0, 0.0, 1.0))
        coverage_plan_marker = self.__create_path_marker_from_path(coverage_plan, 3, ColorRGBA(1.0, 1.0, 0.0, 1.0))

        message = MarkerArray()
        message.markers.append(polygon_marker)
        message.markers.append(path_to_start_marker)
        message.markers.append(coverage_plan_marker)

        for marker in message.markers:
            marker.header.stamp = rospy.get_rostime()
            marker.header.frame_id = "odom"

        self.__visualization_publisher.publish(message)

    def __create_path_marker_from_list(self, path, id, color: ColorRGBA):
        # Create a Marker message for visualization
        marker_msg = Marker()
        marker_msg.type = Marker.LINE_STRIP
        marker_msg.action = Marker.ADD
        marker_msg.scale.x = 0.1  # Line width
        # marker_msg.color.a = 1.0
        # marker_msg.color.r = 1.0
        # marker_msg.color.g = 0.0
        # marker_msg.color.b = 0.0
        marker_msg.color = color
        marker_msg.id = id

        # Populate the Marker message with points
        for point in path:
            position = Point()
            position.x = point[0]
            position.y = point[1]
            marker_msg.points.append(position)

        return marker_msg

    def __create_path_marker_from_path(self, path, id, color: ColorRGBA):
        # Create a Marker message for visualization
        marker_msg = Marker()
        marker_msg.type = Marker.LINE_STRIP
        marker_msg.action = Marker.ADD
        marker_msg.scale.x = 0.1  # Line width
        # marker_msg.color.a = 1.0
        # marker_msg.color.r = 1.0
        # marker_msg.color.g = 0.0
        # marker_msg.color.b = 0.0
        marker_msg.color = color
        marker_msg.id = id

        # Populate the Marker message with points
        for pose in path.poses:
            position = Point()
            position.x = pose.pose.position.x
            position.y = pose.pose.position.x
            marker_msg.points.append(position)

        return marker_msg
