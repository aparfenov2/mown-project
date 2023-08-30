from operator import rshift
from threading import Lock
from typing import Dict

import actionlib
import numpy as np
import rospy
from tf.transformations import quaternion_from_euler

from abstractnode import AbstractNode

from .trajectory_generator import LocalTrajectoryGenerator

from geometry_msgs.msg import (Pose2D, PoseStamped, Point, Quaternion)

from engix_msgs.msg import LocalTrajectoryStamped, Localization, Route


class LocalTrajectoryPlanner(AbstractNode):
    def initialization(self):
        self.lock = Lock()
        self.local_trajectory_generator = LocalTrajectoryGenerator()
        self.ego_pos = None
        self.route = None

        self.__lt_publisher = rospy.Publisher('/local_trajectory_plan', LocalTrajectoryStamped, queue_size=10)

        rospy.Subscriber('/planner/route', Route, self.__route_callback)
        rospy.Subscriber('/planner/localization', Localization, self.__localization_callback)

    def work(self):
        if self.ego_pos is None or self.route is None:
            self.__send_empty_message()
            return

        with self.lock:
            lt = self.local_trajectory_generator.find_loc_trajectory(self.route, self.ego_pos)
            if lt is not None:
                self.__send_local_trajectory(lt)

    def __create_LocalTrajectoryStamped(self, local_trajectory):
        goal = LocalTrajectoryStamped()
        poses = list()
        for traj_point in local_trajectory:
            position = self.__create_Point(traj_point)
            poses.append(position)
            # orientation = self.__create_Quaternion(traj_point)
            # pose_stamped = PoseStamped()
            # pose_stamped.header = rospy.get_rostime()
            # pose_stamped.route = position
            # # pose_stamped.pose.orientation = orientation
            # poses.append(pose_stamped)
        goal.route = poses
        return goal

        # message = LocalTrajectoryStamped()
        # message.header.stamp = rospy.get_rostime()
        
        # route = message.route
        # for point in local_trajectory:
        #     route.append(Pose2D(
        #         x=point[0],
        #         y=point[1],
        #         theta=point[2]
        #     ))
        # return message

    def __create_Point(self, point: Dict):
        dpoint = Pose2D()
        dpoint.x = point[0]
        dpoint.y = point[1]
        return dpoint

    def __create_Quaternion(self, point: Dict):
        quaternion = quaternion_from_euler(0, 0, point[2])
        message = Quaternion()
        message.w = quaternion[0]
        message.x = quaternion[1]
        message.y = quaternion[2]
        message.z = quaternion[3]
        return message

    def __send_local_trajectory(self, local_trajectory):
        goal_trajectory = self.__create_LocalTrajectoryStamped(local_trajectory)
        self.__lt_publisher.publish(goal_trajectory)
        # self.client.send_goal(goal_trajectory)

    def __send_empty_message(self):
        goal = LocalTrajectoryStamped()
        self.__lt_publisher.publish(goal)

    def __localization_callback(self, message):
        # self.print_err("LOCALIZATION CALLBACK {0}", message)
        with self.lock:
            self.ego_pos = np.array([message.pose.x, message.pose.y, message.yaw])

    def __route_callback(self, message):
        # self.print_err("ROUTE CALLBACK {0}", message)
        with self.lock:
            # route_len = len(message.route)
            # self.route = np.zeros((route_len, 2))

            # for i, point in enumerate(message.route):
            #     self.route[i, :] = np.array([point.x, point.y])
            if len(message.route) == 0:
                self.route = None
                return

            self.route = np.array([[p.x, p.y] for p in message.route])

    def print_err(self, print_format, *args, **kwargs):
        rospy.logerr(print_format.format(*args, **kwargs))
