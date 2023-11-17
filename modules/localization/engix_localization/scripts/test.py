#! /usr/bin/env python
import math

import pyproj
import rospy
from tf.transformations import euler_from_quaternion, quaternion_from_euler

from abstractnode import AbstractNode

from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from engix_msgs.msg import Localization 


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


class LocalizationNode(AbstractNode):
    def initialization(self):
        self.projector = Projector()
        self.last_position_gnss = None
        self.last_velocity_gnss = None
        self.last_orientation_imu = None
        self.last_angular_velocity_imu = None

        self.initialization_flag = False

        self.localization_publisher = rospy.Publisher('/planner/localization', Localization, queue_size=1)
        rospy.Subscriber('/fix', NavSatFix, self.gnss_fix_callback)
        rospy.Subscriber('/vel', TwistStamped, self.gnss_vel_callback)
        # rospy.Subscriber('/zed2i/zed_node/odom', Odometry, self.zed_odometry_callback)
        rospy.Subscriber('/zed2i/zed_node/imu/data', Imu, self.zed_imu_callback)

        rospy.loginfo("Initialize LocalizationBridgeNode.")

    def gnss_fix_callback(self, message):
        self.last_position_gnss = self.projector.project((message.longitude, message.latitude))

    def gnss_vel_callback(self, message):
        self.last_velocity_gnss = math.hypot(message.twist.linear.x, message.twist.linear.y)

    def zed_odometry_callback(self, message):
        pass

    def zed_imu_callback(self, message):
        self.last_angular_velocity_imu = message.angular_velocity.z

        orientation_q = message.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
        self.last_orientation_imu = yaw

    def work(self):
        if (self.last_position_gnss is None or self.last_velocity_gnss is None 
            or self.last_orientation_imu is None or self.last_angular_velocity_imu is None):
            return

        if not self.initialization_flag:
            self.initialization_flag = True
            rospy.loginfo("Node LocalizationBridgeNode was initialized. We found gnss and zed camera.")

        localization_message = Localization()
        localization_message.header.stamp = rospy.get_rostime()
        localization_message.pose.x = self.last_position_gnss[2]
        localization_message.pose.y = self.last_position_gnss[3]
        localization_message.yaw = self.last_orientation_imu
        localization_message.speed = self.last_velocity_gnss
        localization_message.angular_speed = self.last_angular_velocity_imu
        localization_message.linear_acceleration = 0.0

        self.localization_publisher.publish(localization_message)
        

if __name__ == '__main__':
    LocalizationNode('LocalizationBridge', 10).run()
