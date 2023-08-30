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
from geometry_msgs.msg import PoseStamped

from engix_msgs.msg import Localization


class OdometryToLocalizationNode(AbstractNode):

    def initialization(self):
        self.localization_publisher = rospy.Publisher(
            rospy.get_param('/planner/topics/localization'),
            Localization,
            queue_size=10
        )
        self.debug_publisher = rospy.Publisher('planner/debug/localization_bridge', PoseStamped, queue_size=2)
        self.br = tf.TransformBroadcaster()
        self._last_time = None
        self._last_speed = None
        self.tf_listener_ = TransformListener()

        # rospy.Subscriber('/laser_odom_to_init', Odometry, self.__odometry_callback) 
        odometry_topic = rospy.get_param('/planner/topics/odometry')
        rospy.Subscriber(odometry_topic, Odometry, self.__odometry_callback)

    def __odometry_callback(self, message: Odometry):
        # if True: # self.tf_listener_.frameExists("/world") and self.tf_listener_.frameExists("/axel_center"):
        #     p1 = PoseStamped()
        #     p1.header.frame_id = "base_link"
        #     # p1.header.frame_id = "axel_center"
        #     p1.pose = message.pose.pose
        #     # p_in_base = self.tf_listener_.transformPose("/axel_center", p1)
        #     p_in_base = self.tf_listener_.transformPose("/base_link", p1)
        #     # rospy.logwarn(f"BEFORE: {p1}\nAFTER: {p_in_base}")
        #     message.pose.pose = p_in_base.pose
        # else:
        #     rospy.logwarn(f"FRAMES doesn't exits: {self.tf_listener_.frameExists('/world')}, {self.tf_listener_.frameExists('/axel_center')}")
        self.debug_publish(message)
        localization_msg = self.__odom_to_loc(message)
        self.localization_publisher.publish(localization_msg)

        # self.send_transform(message)

    def __odom_to_loc(self, odometry: Odometry):
        localization = Localization()
        localization.header = odometry.header
        localization.pose = odometry.pose.pose.position
        q = odometry.pose.pose.orientation
        euler = euler_from_quaternion([q.x, q.y, q.z, q.w])
        localization.yaw = euler[2]
        localization.angular_speed = odometry.twist.twist.angular.z

        # localization.speed = math.hypot(odometry.twist.twist.linear.x, odometry.twist.twist.linear.y)
        velocity = [odometry.twist.twist.linear.x, odometry.twist.twist.linear.y, odometry.twist.twist.linear.z]
        rot_velocity = self.__rotate_to_body_frame(velocity, euler)
        localization.speed = rot_velocity[0]

        if self._last_time is None:
            localization.linear_acceleration = 0.0
        else:
            localization.linear_acceleration = (localization.speed - self._last_speed) / max(10e-10, odometry.header.stamp.to_sec() - self._last_time)

        self._last_time = odometry.header.stamp.to_sec()
        self._last_speed = localization.speed
        return localization

    def __rotate_to_body_frame(self, speed, euler):
        R = euler_matrix(-euler[0], -euler[1], -euler[2])
        t = np.array([1.0]*4)
        t[:3] = np.array(speed)
        t = R.dot(t)
        return t[:3]

    def debug_publish(self, message):
        pose_stamped = PoseStamped()
        pose_stamped.header = message.header
        pose_stamped.pose = message.pose.pose
        self.debug_publisher.publish(pose_stamped)

    def send_transform(self, msg):
        pose = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self.br.sendTransform(
            (pose.x, pose.y, pose.z),
            (q.x, q.y, q.z, q.w),
            msg.header.stamp,
            'base_link',
            "world"
        )


def main():
    node = OdometryToLocalizationNode('OdometryToLocalizationNode', 1)
    node.run()


if __name__ == '__main__':
    main()
