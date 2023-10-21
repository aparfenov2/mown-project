#!/usr/bin/env python3
from time import sleep

from numpy.lib.function_base import append
import rospy
import numpy as np
from task_behavior_engine.tree import Decorator, NodeStatus
from tf.transformations import euler_from_quaternion

from behavior_planning.algorithms import AstarWrapper
from behavior_planning.algorithms import ContinuosAstarWrapper

from geometry_msgs.msg import Point32, PoseStamped
from nav_msgs.msg import Path, OccupancyGrid
from engix_msgs.msg import (Localization, PointWithSpeed, Route,
                             RouteTaskPolygon, RouteTaskToPoint, ProgressRoutePlanner)


class SequenceDecorator(Decorator):
    def __init__(self, name, *args, **kwargs):
        super(SequenceDecorator, self).__init__(name=name,
                                                run_cb=self.run,
                                                *args, **kwargs)

    def run(self, nodedata):
        result = self.node_tick(nodedata)
        if (result == NodeStatus.FAIL):
            return result
        result = self.tick_child()
        return result

    def node_tick(self, nodedata):
        return NodeStatus()
