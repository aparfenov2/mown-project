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


class LocalizationVisualizationNode(AbstractNode):

    def initialization(self):
        self.br = tf.TransformBroadcaster()
        
        self.debug_publisher = rospy.Publisher('/debug/localization_vis', PoseStamped, queue_size=2)

        rospy.Subscriber("/ground_truth/state", Odometry, self.__odometry_callback)

    def __odometry_callback(self, message: Odometry):
        self.debug_publish(message)
        self.send_transform(message)

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
    node = LocalizationVisualizationNode('LocalizationVisualization', 1)
    node.run()


if __name__ == '__main__':
    main()
