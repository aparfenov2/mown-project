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
from move_controller import PIDController, NonlinearController, NonLinearMPC, Frame, PIDSpeedController, Target, SteerMPC, MoveMPC

from coverage_path_planner.area_polygon import AreaPolygon

from std_msgs.msg import Float32
from geometry_msgs.msg import PoseStamped, Pose, Quaternion, TwistStamped, Twist, Vector3
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path

from enginx_msgs.msg import CoverageTask, Localization, RouteTaskPolygon


class CoveragePlannerNode(AbstractNode):
    def initialization(self):
        self._rlock = RLock()

        self._step_size = 3.0
        self._robot_position = []

        self._debug_publisher = rospy.Publisher('planner/debug/coverage_path', Path, queue_size=2)
        self._task_publisher = rospy.Publisher(rospy.get_param('planner/topics/task_polygon_planning'),
                                               RouteTaskPolygon,
                                               queue_size=2)

        rospy.Subscriber(rospy.get_param('/planner/topics/localization'),
                         Localization,
                         self._localization_callback)

        rospy.Subscriber(rospy.get_param('/planner/topics/coverage_task'),
                         CoverageTask,
                         self._coverage_task_callback)

    def _localization_callback(self, message):
        with self._rlock:
            self._robot_position = [message.pose.x,
                                    message.pose.y]

    def _coverage_task_callback(self, message):
        with self._rlock:
            if len(self._robot_position) == 0:
                return

            listed_polygon = [(point.x, point.y) for point in message.target_polygon.points]
            listed_polygon = self._create_offset(listed_polygon)

            if message.approximate:
                temp_polygon = listed_polygon
                listed_polygon = list()

                for i in range(1, len(temp_polygon)):
                    listed_polygon.append(temp_polygon[i-1])
                    point_count = np.linalg.norm(np.array(temp_polygon[i]) - np.array(temp_polygon[i-1]))
                    point_count = max(1, int(point_count))
                    extended_segment = np.linspace(temp_polygon[i-1], temp_polygon[i], point_count).tolist()
                    listed_polygon += extended_segment
            if message.auto_angle:
                path_creator = AreaPolygon(listed_polygon, self._robot_position, ft=self._step_size)
            else:
                path_creator = AreaPolygon(listed_polygon, self._robot_position, ft=self._step_size, angle=message.angle)

            coverage_route_points = path_creator.get_area_coverage()

            path = Path()
            path.header = message.header
            path.header.frame_id = 'world'

            for p in coverage_route_points.coords:
                pose_stamped = PoseStamped()
                pose_stamped.header = message.header
                pose_stamped.pose.position.x = p[0]
                pose_stamped.pose.position.y = p[1]

                path.poses.append(pose_stamped)

            self._debug_publisher.publish(path)

            task = RouteTaskPolygon()
            task.header.stamp = rospy.get_rostime()
            task.path = path
            self._task_publisher.publish(task)

    def _create_offset(self, polygon):
        shapely_polygon = Polygon(polygon)
        poly_line_offset = shapely_polygon.buffer(
            -2.0, resolution=3, join_style=2, mitre_limit=1
        ).exterior
        return poly_line_offset.coords
