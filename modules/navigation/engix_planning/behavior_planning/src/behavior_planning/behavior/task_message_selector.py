#!/usr/bin/env python3
from time import sleep

from numpy.lib.function_base import append
import rospy
import numpy as np
from task_behavior_engine.tree import Behavior, NodeStatus
from tf.transformations import euler_from_quaternion

from behavior_planning.algorithms import AstarWrapper
from behavior_planning.algorithms import ContinuosAstarWrapper
from behavior_planning.node import IdleNode

from geometry_msgs.msg import Point32, PoseStamped
from nav_msgs.msg import Path, OccupancyGrid
from engix_msgs.msg import (Localization, PointWithSpeed, Route,
                             RouteTaskPolygon, RouteTaskToPoint,
                             ProgressRoutePlanner, PlanningTaskType)


class TaskMessageSelector(Behavior):
    def __init__(self, name, frame,
                 default_node=None,
                 coverage_node=None,
                 moving_to_point_node=None,
                 line_moving_node=None,
                 circle_moving_node=None,
                 dubins_planning_node=None,
                 *args, **kwargs):
        super(TaskMessageSelector, self).__init__(name=name,
                                                  run_cb=self.run,
                                                  *args, **kwargs)
        self._frame = frame
        self._default_node = default_node
        self._coverage_node = coverage_node
        self._moving_to_point_node = moving_to_point_node
        self._line_moving_node = line_moving_node
        self._circle_moving_node = circle_moving_node
        self._dubins_planning_node = dubins_planning_node

        self._nodes_list = list()
        self._nodes_list.append(self._default_node)
        self._nodes_list.append(self._coverage_node)
        self._nodes_list.append(self._moving_to_point_node)
        self._nodes_list.append(self._line_moving_node)
        self._nodes_list.append(self._circle_moving_node)
        self._nodes_list.append(self._dubins_planning_node)

    def add_child(self, node):
        raise NotImplementedError("Don't use add_child from SelectByMessageNode.")

    def run(self, nodedata):
        if not self._frame.planning_task_type.has_message():
            rospy.logwarn_throttle(10, "PlanningTaskType has not received.")
            result = self.tick_child(self._default_node)
            return result

        current_task_type = self._frame.planning_task_type.type

        if current_task_type > len(self._nodes_list):
            rospy.logerror_throttle(1, f"PlanningTaskType has wrong type {current_task_type}.")
            result = self.tick_child(self._default_node)
            return result

        node = self._nodes_list[current_task_type]
        result = self.tick_child(node)

        return result
