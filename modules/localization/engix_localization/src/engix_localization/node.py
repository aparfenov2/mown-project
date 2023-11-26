#! /usr/bin/env python
import math

import numpy as np
import rospy
import tf
from tf.transformations import euler_from_quaternion, quaternion_from_euler, quaternion_matrix, quaternion_from_matrix

from abstractnode import AbstractNode



from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from engix_msgs.msg import Localization


import tf2_ros
from geometry_msgs.msg import TransformStamped
from .localization_manager import LocalizationManager
from .utils import create_transform
from .data_projector import DataProjector


class EngixLocalizationNode(AbstractNode):
    def initialization(self):
        self.__is_map_frame_initialized = False
        self.broadcaster = tf.TransformBroadcaster()

        self.initialize_localization()

        self.localization_publisher = rospy.Publisher('/planner/localization',
                                                      Localization,
                                                      queue_size=1)
        rospy.Subscriber('/gps/fix', NavSatFix, self.reach_nav_sat_callback)
        rospy.Subscriber('/gps/vel', TwistStamped, self.reach_vel_callback)
        rospy.Subscriber('/zed2i/zed_node/imu/data', Imu, self.zed_imu_callback)

        # TODO: Remove timer and timer's callback
        # timer_interval = rospy.Duration(1.0)  # 1 Hz

        # Create a timer with the specified interval and callback function
        # self.timer = rospy.Timer(timer_interval, self.timer_callback)

    def timer_callback(self, event):
        # Your callback function code here
        rospy.loginfo("Timer callback executed at %s", rospy.get_time())
        navsat_msg = NavSatFix()
        navsat_msg.header.stamp = rospy.get_rostime()
        navsat_msg.latitude = 0.0
        navsat_msg.longitude = 0.0
        navsat_msg.altitude = 100.0

        self.reach_nav_sat_callback(navsat_msg)

        velocity_msg = TwistStamped()
        velocity_msg.header.stamp = rospy.get_rostime()

        self.reach_vel_callback(velocity_msg)

    def initialize_localization(self):
        listener = tf.TransformListener()
        now = rospy.Time.now()

        base_link_frame = "base_link"
        imu_frame = "zed2i_imu_link"
        gnss_frame = "zed2i_base_link"

        listener.waitForTransform(base_link_frame, imu_frame, now, rospy.Duration(4.0))
        listener.waitForTransform(base_link_frame, gnss_frame, now, rospy.Duration(4.0))

        (trans, rot) = listener.lookupTransform(imu_frame, base_link_frame, now)
        imu2base_link_transform = create_transform(trans, rot)

        (trans, rot) = listener.lookupTransform(gnss_frame, base_link_frame, now)
        gnss2base_link_transform = create_transform(trans, rot)

        data_projector = DataProjector(imu2base_link_transform, gnss2base_link_transform)

        self.__localization = LocalizationManager(
            imu2base_link_transform,
            gnss2base_link_transform,
            data_projector,
            1.0
        )

    def reach_vel_callback(self, message):
        self.__localization.process_velocity_message(message)

    def reach_nav_sat_callback(self, message):
        self.__localization.process_reach_message(message)
        if not self.__is_map_frame_initialized:
            self.initialize_static_transforms(message.header.stamp)
            return

        self.send_map_to_base_link(message.header.stamp)
        self.send_map_to_base_link_footprint(message.header.stamp)
        self.send_localization_message(message.header.stamp)

    def zed_imu_callback(self, message):
        self.__localization.process_zed_imu_message(message)
        if not self.__is_map_frame_initialized:
            self.initialize_static_transforms(message.header.stamp)
            return

    def initialize_static_transforms(self, stamp):
        if self.__localization.contains_gnss() \
                and self.__localization.contains_imu():
            self.send_init_static_transform(stamp)
            self.__is_map_frame_initialized = True

    def send_map_to_base_link(self, stamp):
        position = self.__localization.get_3d_position()
        orientation = self.__localization.get_3d_orientation()
        self.send_transform(position, orientation, stamp, "world", "robot_base_link")

    def send_map_to_base_link_footprint(self, stamp):
        position = self.__localization.get_2d_position()
        orientation = self.__localization.get_2d_orientation()
        self.send_transform(position, orientation, stamp, "world", "robot_base_link_footprint")

    def send_localization_message(self, stamp):
        localization_message = self.__localization.localization
        localization_message.header.stamp = stamp

        self.localization_publisher.publish(localization_message)

    def send_transform(self, position, orientation, stamp, from_frame, to_frame):
        self.broadcaster.sendTransform(position,
                                       orientation,
                                       stamp,
                                       to_frame,
                                       from_frame)

    def send_init_static_transform(self, stamp):
        position = self.__localization.get_3d_position()
        orientation = self.__localization.get_3d_orientation()

        broadcaster = tf2_ros.StaticTransformBroadcaster()
        static_transformStamped = TransformStamped()

        static_transformStamped.header.stamp = stamp
        static_transformStamped.header.frame_id = "world"
        static_transformStamped.child_frame_id = "map"

        static_transformStamped.transform.translation.x = position[0]
        static_transformStamped.transform.translation.y = position[1]
        static_transformStamped.transform.translation.z = position[2]

        static_transformStamped.transform.rotation.x = orientation[0]
        static_transformStamped.transform.rotation.y = orientation[1]
        static_transformStamped.transform.rotation.z = orientation[2]
        static_transformStamped.transform.rotation.w = orientation[3]

        broadcaster.sendTransform(static_transformStamped)
