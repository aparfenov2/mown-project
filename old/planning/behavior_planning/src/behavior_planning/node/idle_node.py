#!/usr/bin/env python3
from time import sleep

from numpy.lib.function_base import append
import rospy
import numpy as np
from task_behavior_engine.tree import Node, NodeStatus
from tf.transformations import euler_from_quaternion

from behavior_planning.algorithms import AstarWrapper
from behavior_planning.algorithms import ContinuosAstarWrapper

from geometry_msgs.msg import Point32, PoseStamped
from nav_msgs.msg import Path, OccupancyGrid
from enginx_msgs.msg import (Localization, PointWithSpeed, Route,
                             RouteTaskPolygon, RouteTaskToPoint, ProgressRoutePlanner)


class IdleNode(Node):
    def __init__(self, name, frame, *args, **kwargs):
        super(IdleNode, self).__init__(name=name,
                                       run_cb=self.run,
                                       *args, **kwargs)
        self._frame = frame

    def run(self, nodedata):
        route = Route()
        self._frame.set_trajectory(route)
        return NodeStatus(NodeStatus.SUCCESS)
