#!/usr/bin/env python3
from time import sleep

from numpy.lib.function_base import append
import rospy
import numpy as np
from task_behavior_engine.tree import Behavior, NodeStatus
from tf.transformations import euler_from_quaternion

from behavior_planning.algorithms import AstarWrapper
from behavior_planning.algorithms import ContinuosAstarWrapper

from geometry_msgs.msg import Point32, PoseStamped
from nav_msgs.msg import Path, OccupancyGrid
from enginx_msgs.msg import (Localization, PointWithSpeed, Route,
                             RouteTaskPolygon, RouteTaskToPoint, ProgressRoutePlanner)


class SelectByMessageNode(Behavior):
    def __init__(self, name, frame,
                 idle_branch,
                 pose_planning_branch,
                 coverage_branch,
                 *args, **kwargs):
        super(SelectByMessageNode, self).__init__(name=name,
                                                  run_cb=self.run,
                                                  *args, **kwargs)
        self._frame = frame
        self._children.append(idle_branch)
        self._children.append(pose_planning_branch)
        self._children.append(coverage_branch)

    def add_child(self, node):
        raise NotImplementedError("Don't use add_child from SelectByMessageNode.")

    def run(self, nodedata):
        if not self._frame.has_localization():
            result = self.tick_idle()
            return result

        # if not self._frame.has_route_task_to_point_message():
        #     result = self.tick_idle()
        #     return result
        if not self._frame.has_route_task_to_point_message() and \
                not self._frame.has_route_task_polygon_message():
            result = self.tick_idle()
            return result

        if self._frame.has_route_task_polygon_message():
            result = self.tick_coverage_planning()
            return result

        result = self.tick_pose_planning()
        return result

    def tick_idle(self):
        child = self._children[0]
        result = self.tick_child(child)
        return result

    def tick_pose_planning(self):
        rospy.logerr("AT tick_pose_planning")
        child = self._children[1]
        result = self.tick_child(child)
        return result

    def tick_coverage_planning(self):
        rospy.logerr("AT tick_coverage_planning")
        child = self._children[2]
        result = self.tick_child(child)
        return result
