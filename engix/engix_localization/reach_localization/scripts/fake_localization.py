#! /usr/bin/env python
import math

import rospy

from abstractnode import AbstractNode

from engix_msgs.msg import Localization 


class FakeLocalizationNode(AbstractNode):
    def initialization(self):
        self.localization_publisher = rospy.Publisher('/planner/localization', Localization, queue_size=1)

    def work(self):
        localization_message = Localization()
        localization_message.header.stamp = rospy.get_rostime()
        localization_message.pose.x = 10.0
        localization_message.pose.y = 10.0
        localization_message.yaw = 0.0
        localization_message.speed = 0.0
        localization_message.angular_speed = 0.0
        localization_message.linear_acceleration = 0.0

        self.localization_publisher.publish(localization_message)
        

if __name__ == '__main__':
    FakeLocalizationNode('FakeLocalizationBridge', 10).run()
