#!/usr/bin/env python3

import numpy as np
from numpy.core.fromnumeric import argmin
import rospy
from threading import Lock
from abstractnode import AbstractNode

from engix_msgs.msg import Localization, LocalTrajectoryStamped


class StateTestNode(AbstractNode):
    def initialization(self):
        self.lock = Lock()
        self.position = (0, 0, 0)
        self.plan = None

        self.state_publisher = rospy.Publisher('/localization', Localization, queue_size=10)
        rospy.Subscriber('/local_trajectory_plan', LocalTrajectoryStamped, self.__loc_traj_plan_callback)

    def work(self):
        with self.lock:
            if self.plan is None or len(self.plan) == 0:
                self.publish_position(self.position)
                return

            p = self.plan.pop(0)

            yaw = np.arctan2(p[1] - self.position[1], p[0] - self.position[0])

            self.position = (p[0], p[1], yaw)
            
            self.publish_position(self.position)

    def publish_position(self, pos):
        x, y, yaw = pos
        msg = Localization()
        msg.header.stamp = rospy.get_rostime()
        msg.pose.x = x
        msg.pose.y = y
        msg.yaw = yaw

        self.state_publisher.publish(msg)

    def __loc_traj_plan_callback(self, msg):
        with self.lock:
            if len(msg.route) > 0:
                plan = np.array([[point.x, point.y] for point in msg.route])
                my_pos = np.array(self.position[:2])
                arg_min = np.argmin(np.linalg.norm(plan - my_pos, axis=1))
                arg_min = arg_min + 1 if (arg_min + 1) < plan.shape[0] - 1 else arg_min

                found_pos = plan[arg_min]
                next_pos = None
                if arg_min < plan.shape[0] - 2:
                    next_pos = plan[arg_min + 1]

                self.plan = plan[arg_min: ].tolist()

                yaw = self.position[2]

                if next_pos is not None:
                    yaw = np.arctan2(next_pos[1] - found_pos[1], next_pos[0] - found_pos[0])

                self.position = (found_pos[0], found_pos[1], yaw)



if __name__ == '__main__':
    StateTestNode('StateTestNode', 10).run()