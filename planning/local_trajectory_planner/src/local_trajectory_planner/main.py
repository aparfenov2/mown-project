from threading import Lock

import numpy as np
import rospy

from abstractnode import AbstractNode

from .trajectory_generator import LocalTrajectoryGenerator

from geometry_msgs.msg import Pose2D
from enginx_msgs.msg import LocalTrajectoryStamped, Localization, Route


class LocalTrajectoryPlanner(AbstractNode):
    def initialization(self):
        self.lock = Lock()
        self.local_trajectory_generator = LocalTrajectoryGenerator()
        self.ego_pos = None
        self.route = None

        self.__lt_publisher = rospy.Publisher('/local_trajectory_plan', LocalTrajectoryStamped, queue_size=10)

        rospy.Subscriber('/route_task', Route, self.__route_callback)
        rospy.Subscriber('/localization', Localization, self.__localization_callback)

    def work(self):
        # rospy.logerr('Pos - {0}, route - {1}'.format(
        #     self.ego_pos, self.route
        # ))
        if self.ego_pos is None or self.route is None:
            return

        with self.lock:
            lt = self.local_trajectory_generator.find_loc_trajectory(self.route, self.ego_pos)
            if lt is not None:
                self.__send_local_trajectory(lt)

    def __create_LocalTrajectoryStamped(self, local_trajectory):
        message = LocalTrajectoryStamped()
        message.header.stamp = rospy.get_rostime()
        
        route = list()
        for point in local_trajectory:
            route.append(Pose2D(
                x=point[0],
                y=point[1],
                theta=point[2]
            ))
        return route

    def __send_local_trajectory(self, local_trajectory):
        route = self.__create_LocalTrajectoryStamped(local_trajectory)
        self.__lt_publisher.publish(route)

    def __localization_callback(self, message):
        self.print_err("LOCALIZATION CALLBACK {0}", message)
        with self.lock:
            self.ego_pos = np.array([message.pose.x, message.pose.y, message.yaw])

    def __route_callback(self, message):
        self.print_err("ROUTE CALLBACK {0}", message)
        with self.lock:
            route_len = len(message.route)
            self.route = np.zeros((route_len, 2))

            for i, point in enumerate(message.route):
                self.route[i, :] = np.array([point.x, point.y])

    def print_err(self, print_format, *args, **kwargs):
        rospy.logerr(print_format.format(*args, **kwargs))
