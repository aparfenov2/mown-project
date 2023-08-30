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


def convert_to_minus_pi_to_pi(angle):
    converted_angle = math.atan2(math.sin(angle), math.cos(angle))
    return converted_angle


class LocalizationBridgeNode(AbstractNode):
    def initialization(self):
        self.__message = Localization()

        self.localization_publisher = rospy.Publisher(
            rospy.get_param("~topics/localization"), Localization, queue_size=10
        )

        self.listener = tf.TransformListener()
        self.target_frame = rospy.get_param("~target_frame")
        self.source_frame = rospy.get_param("~source_frame")

        self.__message.header.frame_id = self.target_frame

        self.__previous_position = None
        self.__previous_time = None
        self.__previous_angle = None

        now = rospy.Time.now()
        self.listener.waitForTransform(
            self.target_frame, self.source_frame, now, rospy.Duration(50.0)
        )

    def work(self):
        try:
            # Lookup the transformation between 'target_frame' and 'source_frame'
            # (trans, rot) = self.listener.lookupTransform(self.target_frame, self.source_frame, rospy.Time(0))
            now = rospy.Time.now()
            self.listener.waitForTransform(
                self.target_frame, self.source_frame, now, rospy.Duration(0.1)
            )
            (trans, rot) = self.listener.lookupTransform(
                self.target_frame, self.source_frame, now
            )
            cur_time = rospy.get_rostime()
            localization = self.__to_localization(trans, rot, cur_time)
            self.localization_publisher.publish(localization)
        except (
            tf.LookupException,
            tf.ConnectivityException,
            tf.ExtrapolationException,
        ):
            rospy.logwarn("Error while looking up transform")

    def __to_localization(self, trans, rot, current_time) -> Localization:
        self.__message.pose.x = trans[0]
        self.__message.pose.y = trans[1]
        self.__message.pose.z = trans[2]

        (_, _, yaw) = euler_from_quaternion(rot)
        speed = self.__calculate_speed(trans, current_time.to_sec())
        angular_speed = self.__calculate_angular_speed(yaw, current_time.to_sec())

        self.__message.yaw = yaw

        if angular_speed != 0.0:
            self.__message.angular_speed = angular_speed

        if speed != 0.0:
            self.__message.speed = speed

        self.__message.header.stamp = current_time

        self.__previous_time = current_time.to_sec()

        return self.__message

    def __calculate_speed(self, current_position, cur_time):
        if self.__previous_position is None or self.__previous_time is None:
            self.__previous_position = current_position
            return 0.0

        delta_position = [
            current_position[0] - self.__previous_position[0],
            current_position[1] - self.__previous_position[1],
            current_position[2] - self.__previous_position[2],
        ]
        delta_time = cur_time - self.__previous_time

        if delta_time == 0:
            speed = 0.0
        else:
            speed = math.hypot(*delta_position) / delta_time

        self.__previous_position = current_position
        return speed

    def __calculate_angular_speed(self, current_angle, current_time):
        if self.__previous_angle is None or self.__previous_time is None:
            self.__previous_angle = current_angle
            return 0.0

        delta_angle = current_angle - self.__previous_angle
        delta_angle = convert_to_minus_pi_to_pi(delta_angle)
        delta_time = current_time - self.__previous_time

        # Ensure that the delta_time is not zero to avoid division by zero
        self.__previous_angle = current_angle

        if delta_time != 0:
            angular_velocity = delta_angle / delta_time
            return angular_velocity
        else:
            return 0.0


if __name__ == "__main__":
    LocalizationBridgeNode("LocalizationBridgeNode", 20).run()
