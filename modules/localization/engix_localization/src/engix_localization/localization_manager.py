#! /usr/bin/env python
import math
from typing import Tuple

import numpy as np
import rospy
import tf
from tf.transformations import euler_from_quaternion, quaternion_from_euler, quaternion_matrix, quaternion_from_matrix

from abstractnode import AbstractNode

from .wgs_convertor import convert_to_utm
from .utils import quaternion2list, create_transform

from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import TwistStamped, Quaternion
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from engix_msgs.msg import Localization


import tf2_ros
from geometry_msgs.msg import TransformStamped
from .kalman_filter import KalmanFilter


class LocalizationManager:
    def __init__(self, imu_to_base_transform, gnss_to_base_transform, data_projector, dt) -> None:
        self.__data_projector = data_projector

        self.__gnss_received = False
        self.__imu_received = False
        self.__speed_received = False

        self.__kalman_filter = KalmanFilter(2, 1)
        self.__kalman_filter.F = np.array([[1., dt],
                                           [0., 1.]])
        self.__kalman_filter.Q = np.array([[0.1, 0.0],
                                           [0.0, 0.1]])

        self.__kalman_filter.H = np.array([[1., 0.]])
        self.__kalman_filter.R = np.array([[1.]])

        self.imu_to_base_transform = imu_to_base_transform
        self.gnss_to_base_transform = gnss_to_base_transform

        self.__localization = Localization()
        self.__orientation = None

    def process_reach_message(self, message):
        self.__gnss_received = True

        # TODO: remove old code
        # coords = convert_to_utm(message.latitude, message.longitude)

        try:
            coords = convert_to_utm(message.latitude, message.longitude)
        except Exception as e:
            rospy.logerr(f"Error converting coordinates: {e}")
            return

        position = self.__data_projector.gnss_to_base_position(
            coords[0], coords[1], message.altitude)

        self.__localization.pose.x = position[0]
        self.__localization.pose.y = position[1]
        self.__localization.pose.z = position[2]

    def process_zed_imu_message(self, message: Imu):
        self.__imu_received = True

        self.__localization.angular_speed = message.angular_velocity.z

        yaw, orientation = self.__data_projector.yaw_from_imu(
            message.orientation)
        self.__localization.yaw = yaw
        self.__orientation = orientation

    def process_velocity_message(self, message: TwistStamped):
        self.__speed_received = True

        speed = math.hypot(message.twist.linear.x, message.twist.linear.y)
        self.__kalman_filter.predict()
        self.__kalman_filter.update(speed)

        speed, acceleration = self.__kalman_filter.state
        self.__localization.speed = speed
        self.__localization.linear_acceleration = acceleration

    @property
    def localization(self) -> Localization:
        return self.__localization

    def contains_gnss(self):
        return self.__gnss_received

    def contains_imu(self):
        return self.__imu_received

    def contains_speed(self):
        return self.__speed_received

    def get_3d_position(self):
        position = (
            self.__localization.pose.x,
            self.__localization.pose.y,
            self.__localization.pose.z
        )
        return position

    def get_3d_orientation(self):
        return self.__orientation

    def get_2d_position(self):
        position = (
            self.__localization.pose.x,
            self.__localization.pose.y,
            0.0
        )
        return position

    def get_2d_orientation(self):
        yaw = self.__localization.yaw
        q = quaternion_from_euler(0.0, 0.0, yaw)
        return q
