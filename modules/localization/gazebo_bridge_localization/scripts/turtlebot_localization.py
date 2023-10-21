#!/usr/bin/env python3
import math
from threading import RLock
import numpy as np

import rospy
import tf
from tf import TransformListener
from tf.transformations import euler_from_quaternion, euler_matrix

from abstractnode import AbstractNode
from nav_msgs.msg import Odometry

from engix_msgs.msg import Localization


class LocalizationBridgeNode(AbstractNode):
    def initialization(self):
        self.localization_publisher = rospy.Publisher(
            rospy.get_param('~topics/localization'),
            Localization,
            queue_size=10
        )
        self.br = tf.TransformBroadcaster()

        self._last_speed = None
        self._last_time = None

        rospy.Subscriber(rospy.get_param('~topics/odometry'),
                         Odometry, self.odometry_callback)

    def odometry_callback(self, message):
        localization_msg = self.__odom_to_loc(message)
        self.localization_publisher.publish(localization_msg)

    def __odom_to_loc(self, odometry: Odometry):
        localization = Localization()
        localization.header = odometry.header
        localization.pose = odometry.pose.pose.position
        q = odometry.pose.pose.orientation
        euler = euler_from_quaternion([q.x, q.y, q.z, q.w])
        localization.yaw = euler[2]
        localization.angular_speed = odometry.twist.twist.angular.z

        velocity = [odometry.twist.twist.linear.x,
                    odometry.twist.twist.linear.y, odometry.twist.twist.linear.z]
        # rot_velocity = self.__rotate_to_body_frame(velocity, euler)
        localization.speed = math.hypot(velocity[0], velocity[1], velocity[2])

        if self._last_time is None:
            localization.linear_acceleration = 0.0
        else:
            localization.linear_acceleration = (localization.speed - self._last_speed) / max(
                10e-10, odometry.header.stamp.to_sec() - self._last_time)

        self._last_time = odometry.header.stamp.to_sec()
        self._last_speed = localization.speed
        return localization


if __name__ == '__main__':
    LocalizationBridgeNode('LocalizationBridgeNode', 5).run()
