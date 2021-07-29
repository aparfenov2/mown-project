#!/usr/bin/env python3
from threading import RLock

import rospy
from tf.transformations import euler_from_quaternion

from abstractnode import AbstractNode
from nav_msgs.msg import Odometry

from enginx_msgs.msg import Localization


class OdometryToLocalizationNode(AbstractNode):
    
    def initialization(self):
        self.localization_publisher = rospy.Publisher('/localization', Localization, queue_size=10)

        # rospy.Subscriber('/laser_odom_to_init', Odometry, self.__odometry_callback) 
        odometry_topic = rospy.get_param('/planning/odometry', '/ground_truth/state')
        rospy.Subscriber(odometry_topic, Odometry, self.__odometry_callback)

    def __odometry_callback(self, message: Odometry):

        localization_msg = self.__odom_to_loc(message)
        self.localization_publisher.publish(localization_msg)

    def __odom_to_loc(self, odometry: Odometry):
        localization = Localization()
        localization.header = odometry.header
        localization.pose = odometry.pose.pose.position
        q = odometry.pose.pose.orientation
        euler = euler_from_quaternion([q.x, q.y, q.z, q.w])
        localization.yaw = euler[2]
        localization.speed = odometry.twist.twist.linear.x

        return localization


def main():
    node = OdometryToLocalizationNode('OdometryToLocalizationNode', 1)
    node.run()


if __name__ == '__main__':
    main()
