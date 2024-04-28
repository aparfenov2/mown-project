#!/usr/bin/env python3
import math
from threading import RLock
import numpy as np

import rospy
import tf
from tf import TransformListener
from tf.transformations import euler_from_quaternion, euler_matrix

from abstractnode import AbstractNode
from engix_localization.wgs_convertor import convert_to_utm
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Vector3Stamped

from engix_msgs.msg import Localization


class LocalizationBridgeNode(AbstractNode):
    def initialization(self):
        self.localization_publisher = rospy.Publisher(
            rospy.get_param("~topics/localization"), Localization, queue_size=10
        )
        self.br = tf.TransformBroadcaster()

        self._last_speed = None
        self._last_time = None

        self._got_odometry = False
        self._got_gnss_velocity = False

        self._localization = Localization()

        rospy.Subscriber(
            rospy.get_param("~topics/odometry"), Odometry, self.odometry_callback
        )
        rospy.Subscriber("/gps/fix", NavSatFix, self.gnss_callback)
        rospy.Subscriber(
            "/gps/fix_velocity",
            Vector3Stamped,
            self.gnss_velocity_callback,
        )

    def odometry_callback(self, message: Odometry):
        self._got_odometry = True
        q = message.pose.pose.orientation
        euler = euler_from_quaternion([q.x, q.y, q.z, q.w])
        self._localization.yaw = euler[2]
        self._localization.angular_speed = message.twist.twist.angular.z

    def gnss_callback(self, message: NavSatFix) -> None:
        if not self._got_odometry or not self._got_gnss_velocity:
            return

        coords = convert_to_utm(message.latitude, message.longitude)
        self._localization.pose.x = coords[0]
        self._localization.pose.y = coords[1]
        self._localization.pose.z = message.altitude
        self.localization_publisher.publish(self._localization)

    def gnss_velocity_callback(self, message: Vector3Stamped) -> None:
        self._got_gnss_velocity = True

        velocity = [message.vector.x, message.vector.y, message.vector.z]
        self._localization.speed = math.hypot(velocity[0], velocity[1], velocity[2])
        if self._last_time is None:
            self._localization.linear_acceleration = 0.0
        else:
            self._localization.linear_acceleration = (
                self._localization.speed - self._last_speed
            ) / max(10e-10, message.header.stamp.to_sec() - self._last_time)

        self._last_time = message.header.stamp.to_sec()
        self._last_speed = self._localization.speed

    def __odom_to_loc(self, odometry: Odometry):
        localization = Localization()
        localization.header = odometry.header
        localization.pose = odometry.pose.pose.position
        q = odometry.pose.pose.orientation
        euler = euler_from_quaternion([q.x, q.y, q.z, q.w])
        localization.yaw = euler[2]
        localization.angular_speed = odometry.twist.twist.angular.z

        velocity = [
            odometry.twist.twist.linear.x,
            odometry.twist.twist.linear.y,
            odometry.twist.twist.linear.z,
        ]
        # rot_velocity = self.__rotate_to_body_frame(velocity, euler)
        localization.speed = math.hypot(velocity[0], velocity[1], velocity[2])

        if self._last_time is None:
            localization.linear_acceleration = 0.0
        else:
            localization.linear_acceleration = (
                localization.speed - self._last_speed
            ) / max(10e-10, odometry.header.stamp.to_sec() - self._last_time)

        self._last_time = odometry.header.stamp.to_sec()
        self._last_speed = localization.speed
        return localization


if __name__ == "__main__":
    LocalizationBridgeNode("LocalizationBridgeNode", 5).run()
