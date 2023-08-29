#!/usr/bin/env python3
from math import cos
from threading import RLock

import rospy
import actionlib
import numpy as np
from scipy import optimize
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from shapely.geometry import Polygon

from abstractnode import AbstractNode
from move_controller import (
    PIDController,
    NonlinearController,
    NonLinearMPC,
    Frame,
    PIDSpeedController,
    Target,
    SteerMPC,
    MoveMPC,
)

from coverage_path_planner.area_polygon import AreaPolygon
from coverage_path_planner.coverage_path_plannint_algorithms import (
    CoveragePathPlanningAlgorithm,
)

from std_msgs.msg import Float32
from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Quaternion,
    TwistStamped,
    Twist,
    Vector3,
)
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path

from engix_msgs.msg import (
    CoverageTask,
    Localization,
    RouteTaskPolygon,
    CoveragePlanningTask,
    PlanningTaskType,
)
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, Pose, Quaternion


class CoveragePlannerNode(AbstractNode):
    def initialization(self):
        self._rlock = RLock()

        self._step_size = rospy.get_param("~step_size")
        self._offset_distance = rospy.get_param("~offset_distance")
        self._offset_resolution = rospy.get_param("~offset_resolution")
        self._robot_position = []

        self.turning_radius = 3.0
        self.target_speed = 0.2

        self.__algo = CoveragePathPlanningAlgorithm(
            self._offset_distance, self._offset_resolution, self._step_size
        )

        # self._debug_publisher = rospy.Publisher(
        #     "planner/debug/coverage_path", Path, queue_size=2
        # )
        # self._task_publisher = rospy.Publisher(
        #     rospy.get_param("planner/topics/task_polygon_planning"),
        #     RouteTaskPolygon,
        #     queue_size=2,
        # )

        # self.task_type_publisher = rospy.Publisher(
        #     rospy.get_param("/planner/topics/behavior_planner/type_task"),
        #     PlanningTaskType,
        #     queue_size=1,
        #     latch=True,
        # )
        # self.task_publisher = rospy.Publisher(
        #     rospy.get_param("/planner/topics/behavior_planner/coverage_planning_task"),
        #     CoveragePlanningTask,
        #     queue_size=1,
        #     latch=True,
        # )
        self.visualization_publisher = rospy.Publisher(
            "/coverage_planner/visualization",
            MarkerArray,
            queue_size=1,
        )

        rospy.Subscriber(
            rospy.get_param("~topics/localization"),
            Localization,
            self._localization_callback,
        )

        # rospy.Subscriber(
        #     rospy.get_param("/planner/topics/coverage_task"),
        #     CoverageTask,
        #     self._coverage_task_callback,
        # )

        rospy.Subscriber(
            "coverage_planner/visualization",
            CoverageTask,
            self.__visualization_task,
        )

    def _localization_callback(self, message):
        with self._rlock:
            self._robot_position = [message.pose.x, message.pose.y]

    def __visualization_task(self, message: CoverageTask) -> None:
        if len(self._robot_position) == 0:
            return

        listed_polygon = [(point.x, point.y) for point in message.target_polygon.points]
        path = self.__algo.plan(
            self._robot_position,
            listed_polygon,
            message.approximate,
            message.auto_angle,
            message.angle,
        )
        message = self.__create_markers(listed_polygon, path)

        for marker in message.markers:
            marker.header.stamp = message.header.stamp
            marker.header.frame_id = "map"

        self.visualization_publisher.publish(message)

    def __create_markers(self, polygon, path) -> MarkerArray:
        new_polygon = list(polygon)
        new_polygon.append(polygon[0])
        path_marker = self.__create_path_marker(path)
        polygon_marker = self.__create_path_marker(polygon)

        message = MarkerArray()
        message.markers.append(path_marker)
        message.markers.append(polygon_marker)
        return message

    def __create_path_marker(path):
        # Create a Marker message for visualization
        marker_msg = Marker()
        marker_msg.type = Marker.LINE_STRIP
        marker_msg.action = Marker.ADD
        marker_msg.scale.x = 0.1  # Line width
        marker_msg.color.a = 1.0
        marker_msg.color.r = 1.0
        marker_msg.color.g = 0.0
        marker_msg.color.b = 0.0

        # Populate the Marker message with points
        for point in path:
            position = Point()
            position.x = point[0]
            position.y = point[1]
            marker_msg.points.append(position)

        return marker_msg

    def _coverage_task_callback(self, message):
        with self._rlock:
            if len(self._robot_position) == 0:
                return

            listed_polygon = [
                (point.x, point.y) for point in message.target_polygon.points
            ]
            listed_polygon = self._create_offset(listed_polygon)

            if message.approximate:
                temp_polygon = listed_polygon
                listed_polygon = list()

                for i in range(1, len(temp_polygon)):
                    listed_polygon.append(temp_polygon[i - 1])
                    point_count = np.linalg.norm(
                        np.array(temp_polygon[i]) - np.array(temp_polygon[i - 1])
                    )
                    point_count = max(1, int(point_count))
                    extended_segment = np.linspace(
                        temp_polygon[i - 1], temp_polygon[i], point_count
                    ).tolist()
                    listed_polygon += extended_segment
            print(listed_polygon)
            if message.auto_angle:
                path_creator = AreaPolygon(
                    listed_polygon, self._robot_position, ft=self._step_size
                )
            else:
                path_creator = AreaPolygon(
                    listed_polygon,
                    self._robot_position,
                    ft=self._step_size,
                    angle=message.angle,
                )

            coverage_route_points = path_creator.get_area_coverage()

            path = Path()
            path.header = message.header
            path.header.frame_id = "world"

            for p in coverage_route_points.coords:
                pose_stamped = PoseStamped()
                pose_stamped.header = message.header
                pose_stamped.pose.position.x = p[0]
                pose_stamped.pose.position.y = p[1]

                path.poses.append(pose_stamped)

            self._debug_publisher.publish(path)

            time_now = rospy.Time.now()
            coverage_task_message = CoveragePlanningTask()
            coverage_task_message.header.stamp = time_now
            coverage_task_message.turning_radius = self.turning_radius
            coverage_task_message.step_size = 0.1
            coverage_task_message.target_speed = self.target_speed
            coverage_task_message.path = path

            self._publish_task_type(time_now)
            self.task_publisher.publish(coverage_task_message)

    def _publish_task_type(self, stamp):
        planning_task_type_message = PlanningTaskType()
        planning_task_type_message.header.stamp = stamp
        planning_task_type_message.type = PlanningTaskType.COVERAGE_TASK
        self.task_type_publisher.publish(planning_task_type_message)

    def _create_offset(self, polygon):
        shapely_polygon = Polygon(polygon)
        poly_line_offset = shapely_polygon.buffer(
            self._offset_distance,
            resolution=self._offset_resolution,
            join_style=2,
            mitre_limit=1,
        ).exterior
        return poly_line_offset.coords
