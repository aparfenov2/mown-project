#!/usr/bin/env python3
import time 

import numpy as np
import rospy

from engix_msgs.msg import Localization, Route, PointWithSpeed



class SimpleTest:
    def __init__(self, length=10.0, dx=0.5):
        rospy.init_node('simple_test', anonymous=True)
        self.state = None
        self.length = length
        self.dx = dx


        self.control_publisher = rospy.Publisher('/route', Route, queue_size=10)

        rospy.Subscriber('/localization', Localization, self.state_callback)

    def state_callback(self, message):
        self.state = message

    def generate_path(self, pose, yaw):
        dyaw = [np.pi, np.pi/5.0, -np.pi/2.0]
        yaw = np.random.choice(dyaw) + yaw

        route = []

        for i in range(int(self.length / self.dx)):
            dx, dy = self.dx * (i + 1.0) * np.math.cos(yaw), self.dx * (i + 1.0) * np.math.sin(yaw)
            new_pose = pose + np.array([dx, dy])  
            pws = PointWithSpeed()
            pws.x = new_pose[0]
            pws.y = new_pose[1]
            pws.speed = 0.5
            route.append(pws)

        route[-1].speed = 0.0

        message = Route()
        message.header.stamp = rospy.get_rostime()
        message.route = route
        return message

    def run_test(self):
        while self.state is None:
            time.sleep(0.1)
        
        state = np.array([self.state.pose.x, self.state.pose.y])
        yaw = self.state.yaw
        message = self.generate_path(state, yaw)
        self.control_publisher.publish(message)


if __name__ == "__main__":
    SimpleTest(length=5.0, dx=0.1).run_test()