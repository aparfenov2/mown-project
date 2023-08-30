#!/usr/bin/env python3

import threading
import rospy

from abstractnode import AbstractNode
from global_planner.common.frame import Frame
from global_planner.planners.straight_line_planner import StraightLinePlanner
from global_planner.planners.dubins_path_planner import DubinsPathPlanner

from nav_msgs.msg import Path
from engix_msgs.msg import Localization, LineMovingTask, DubinsPlanningTask


class GlobalPlanningNode(AbstractNode):
    def initialization(self):
        self._rlock = threading.RLock()
        self._frame = Frame()
        self._straight_line_planner = StraightLinePlanner()
        self._dubins_planning_task_planner = DubinsPathPlanner()

        self._path_publisher = rospy.Publisher(
            rospy.get_param('~topics/result_path'),
            Path,
            queue_size=10
        )

        rospy.Subscriber(rospy.get_param('~topics/localization'),
                         Localization,
                         self.localization_callback)
        rospy.Subscriber(rospy.get_param('~topics/straight_line_task'),
                         LineMovingTask,
                         self.straight_line_task_callback)
        rospy.Subscriber(rospy.get_param('~topics/dubins_planning_task'),
                         DubinsPlanningTask,
                         self.dubins_planning_task_callback)

    def localization_callback(self, message):
        with self._rlock:
            self._frame.set_localization(message)

    def straight_line_task_callback(self, message):
        with self._rlock:
            self._frame.set_line_moving_task(message)
            path = self._straight_line_planner.plan(self._frame)
            self.send_path(path)

    def dubins_planning_task_callback(self, message):
        with self._rlock:
            self._frame.set_dubins_planning_task(message)
            path = self._dubins_planning_task_planner.plan(self._frame)
            self.send_path(path)

    def send_path(self, path):
        path.header.frame_id = 'odom'
        path.header.stamp = rospy.get_rostime()

        for pose in path.poses:
            pose.header = path.header

        self._path_publisher.publish(path)

