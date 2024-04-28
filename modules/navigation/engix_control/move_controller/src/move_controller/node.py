#!/usr/bin/env python3

import threading
import rospy

from abstractnode import AbstractNode
from move_controller.algorithms.pid_base_controller import PIDTrackingController
from move_controller.common.frame import Frame

from nav_msgs.msg import Path
from engix_msgs.msg import Localization
from geometry_msgs.msg import Twist


class ControlNode(AbstractNode):
    def initialization(self):
        self._rlock = threading.RLock()
        self._frame = Frame()

        linear_controller_params = {
            'kp': rospy.get_param('~pid_tracking_controller/linear_pid/kp'),
            'ki': rospy.get_param('~pid_tracking_controller/linear_pid/ki'),
            'kd': rospy.get_param('~pid_tracking_controller/linear_pid/kd')
        }

        angular_controller_params = {
            'kp': rospy.get_param('~pid_tracking_controller/angular_pid/kp'),
            'ki': rospy.get_param('~pid_tracking_controller/angular_pid/ki'),
            'kd': rospy.get_param('~pid_tracking_controller/angular_pid/kd')
        }

        max_linear_velocity = rospy.get_param('~pid_tracking_controller/max_linear_velocity')
        max_acceleration = rospy.get_param('~pid_tracking_controller/max_acceleration')
        max_angular_velocity = rospy.get_param('~pid_tracking_controller/max_angular_velocity')

        self.max_linear_velocity = max_linear_velocity
        self.d_linear_velocity = rospy.get_param('~pid_tracking_controller/d_linear_velocity')

        self._controller = PIDTrackingController(
            linear_controller_params,
            angular_controller_params,
            max_linear_velocity,
            max_acceleration,
            max_angular_velocity,
            2
        )

        self._twist_publisher = rospy.Publisher(
            rospy.get_param('~topics/twist'),
            Twist,
            queue_size=10
        )

        rospy.Subscriber(rospy.get_param('~topics/localization'),
                         Localization,
                         self.localization_callback)

        rospy.Subscriber(rospy.get_param('~topics/path'),
                         Path,
                         self.path_callback)

    def localization_callback(self, message):
        with self._rlock:
            self._frame.set_localization(message)
            self.control()

    def path_callback(self, message):
        with self._rlock:
            self._frame.set_path(message)
            speed_profile = self.generate_speed_profile()
            self._frame.set_speed_profile(speed_profile)

    def generate_speed_profile(self):
        path = self._frame.path.get_path_as_array()
        speeds = [self.max_linear_velocity] * len(path)
        for i in range(1, len(speeds) + 1):
            speeds[-i] = min((i - 1) * self.d_linear_velocity, speeds[-i])

        return speeds
    
    def work(self):
        self.control()

    def control(self):
        lin_vel, ang_vel = self._controller.control(self._frame)
        # print(lin_vel, ang_vel)

        # linear_velocity = self._frame.localization.speed
        # linear_velocity = linear_velocity + acc

        message = Twist()
        message.linear.x = lin_vel
        message.angular.z = ang_vel
        self._twist_publisher.publish(message)
