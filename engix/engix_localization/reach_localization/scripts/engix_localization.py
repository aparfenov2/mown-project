#! /usr/bin/env python
import math

import numpy as np
import pyproj
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

# Localization:
#   ZED
#   REACH
# Orientation:
#   ZED
# Speed:
#   ZED
# Angular speed:
#   ZED


class Projector:
    def __init__(self) -> None:
        self._projections = {}

    def zone(self, coordinates):
        if 56 <= coordinates[1] < 64 and 3 <= coordinates[0] < 12:
            return 32
        if 72 <= coordinates[1] < 84 and 0 <= coordinates[0] < 42:
            if coordinates[0] < 9:
                return 31
            elif coordinates[0] < 21:
                return 33
            elif coordinates[0] < 33:
                return 35
            return 37
        return int((coordinates[0] + 180) / 6) + 1

    def letter(self, coordinates):
        return 'CDEFGHJKLMNPQRSTUVWXX'[int((coordinates[1] + 80) / 8)]

    def project(self, coordinates):
        z = self.zone(coordinates)
        l = self.letter(coordinates)
        if z not in self._projections:
            self._projections[z] = pyproj.Proj(proj='utm', zone=z, ellps='WGS84')
        x, y = self._projections[z](coordinates[0], coordinates[1])
        if y < 0:
            y += 10000000
        return z, l, x, y

    def unproject(self, proj, l, coords):
        if l < 'N':
            for p in coords:
                p[1] -= 10000000
        wgs = [proj(p[0], p[1], inverse=True) for p in coords]
        return wgs


def quaternion2list(quaternion):
    return [quaternion.x, quaternion.y, quaternion.z, quaternion.w]


def create_transform(trans, rot):
    pass


class LocalizationManager:
    def __init__(self, imu_to_base_transform, gnss_to_base_transform) -> None:
        self.projector = Projector()

        self._gnss_received = False
        self._imu_received = False

        self.imu_to_base_transform = imu_to_base_transform
        self.gnss_to_base_transform = gnss_to_base_transform

        self.position = None
        self.orientation = None
        self.speed = 0.0

    def process_reach_message(self, message):
        self._gnss_received = True

        coords = self.projector.project((message.longitude, message.latitude))

        translate_mat = np.ones(shape=(4, 4))
        translate_mat[0, 3] = coords[2]
        translate_mat[1, 3] = coords[3]
        translate_mat[2, 3] = message.altitude

        base_link = np.dot(translate_mat, self.gnss_to_base_transform)
        self.position = [base_link[0, 3], base_link[1, 3], base_link[2, 3]]

    def process_zed_imu_message(self, message):
        self._imu_received = True

        self.angular_velocity = message.angular_velocity.z

        orientation = quaternion2list(message.orientation)
        rot_mat = quaternion_matrix(orientation)
        base_rot = np.dot(rot_mat, self.imu_to_base_transform)

        self.orientation = quaternion_from_matrix(base_rot)

    def process_velocity_message(self, message):
        self.speed = math.hypot(message.twist.linear.x, message.twist.linear.y)

    def contains_gnss(self):
        return self._gnss_received

    def contains_imu(self):
        return self._imu_received

    def get_3d_position(self):
        return self.position

    def get_3d_orientation(self):
        return self.orientation

    def get_2d_position(self):
        position_2d = [self.position[0], self.position[1], 0.0]
        return position_2d

    def get_2d_orientation(self):
        yaw = self.get_yaw()
        q = quaternion_from_euler(0.0, 0.0, yaw)
        return q

    def get_yaw(self):
        (_, _, yaw) = euler_from_quaternion(self.orientation)
        return yaw

    def get_speed(self):
        return self.speed

    def get_angular_speed(self):
        return self.angular_velocity


class EngixLocalizationNode(AbstractNode):
    def initialization(self):
        self._is_map_frame_initialized = False
        self.broadcaster = tf.TransformBroadcaster()

        self.initialize_localization()

        self.localization_publisher = rospy.Publisher('/planner/localization',
                                                      Localization,
                                                      queue_size=1)
        rospy.Subscriber('/fix', NavSatFix, self.reach_nav_sat_callback)
        rospy.Subscriber('/vel', TwistStamped, self.reach_vel_callback)
        rospy.Subscriber('/zed2i/zed_node/imu/data', Imu, self.zed_imu_callback)

    def initialize_localization(self):
        listener = tf.TransformListener()
        now = rospy.Time.now()

        base_link_frame = "base_link"
        imu_frame = "imu_frame"
        gnss_frame = "gnss_frame"

        listener.waitForTransform(base_link_frame, imu_frame, now, rospy.Duration(4.0))
        listener.waitForTransform(base_link_frame, gnss_frame, now, rospy.Duration(4.0))

        (trans, rot) = listener.lookupTransform(imu_frame, base_link_frame, now)
        imu2base_link_transform = create_transform(trans, rot)

        (trans, rot) = listener.lookupTransform(gnss_frame, base_link_frame, now)
        gnss2base_link_transform = create_transform(trans, rot)

        self._localization = LocalizationManager(
            imu2base_link_transform,
            gnss2base_link_transform
        )

    def reach_vel_callback(self, message):
        self._localization.process_velocity_message(message)

    def reach_nav_sat_callback(self, message):
        self._localization.process_reach_message(message)
        if not self._is_map_frame_initialized:
            self.initialize_static_transforms()
            return

        self.send_map_to_base_link(message.header.stamp)
        self.send_map_to_base_link_footprint(message.header.stamp)
        self.send_localization_message(message.header.stamp)
        # send: 
        # Localization Message
        # map to base_link transform
        # map to base_link_footprint transform

    def zed_imu_callback(self, message):
        self._localization.process_zed_imu_message(message)
        if not self._is_map_frame_initialized:
            self.initialize_static_transforms()
            return

    def initialize_static_transforms(self):
        if self._localization.contains_gnss() \
                and self._localization.contains_imu():
            map_to_init_transform = self._localization.get_3d_localization()
            self.send_init_static_transform(map_to_init_transform)
            self._is_map_frame_initialized = True

    def send_map_to_base_link(self, stamp):
        position = self._localization.get_3d_position()
        orientation = self._localization.get_3d_orientation()
        self.send_transform(position, orientation, stamp, "world", "base_link")

    def send_map_to_base_link_footprint(self, stamp):
        position = self._localization.get_2d_position()
        orientation = self._localization.get_2d_orientation()
        self.send_transform(position, orientation, stamp, "world", "base_link_footprint")

    def send_localization_message(self, stamp):
        position = self._localization.get_2d_position()
        yaw = self._localization.get_yaw()
        speed = self._localization.get_speed()
        angular_speed = self._localization.get_angular_speed()

        localization_message = Localization()
        localization_message.header.stamp = rospy.get_rostime()
        localization_message.pose.x = position[0]
        localization_message.pose.y = position[1]
        localization_message.yaw = yaw
        localization_message.speed = speed
        localization_message.angular_speed = angular_speed
        localization_message.linear_acceleration = 0.0

        self.localization_publisher.publish(localization_message)

    def send_transform(self, position, orientation, stamp, from_frame, to_frame):
        self.broadcaster.sendTransform(position,
                                       orientation,
                                       stamp,
                                       to_frame,
                                       from_frame)

    def send_init_static_transform(self, stamp):
        position = self._localization.get_3d_position()
        orientation = self._localization.get_3d_orientation()

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
