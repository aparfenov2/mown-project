#!/usr/bin/env python3

from time import sleep
from numpy.lib.function_base import append
import rospy
import numpy as np

from abstractnode import AbstractNode
from continious_state_planner import AstarPathPlanner, GridGenerator, AStar
from coverage_path_planner import CoveragePathClient
from path_planning import Frame, CoveragePlanner, APathPlanner

from engix_msgs.msg import Localization, PointWithSpeed, Route, RouteTaskPolygon, RouteTaskToPoint, ProgressRoutePlanner
from geometry_msgs.msg import Point32, PoseStamped
from nav_msgs.msg import Path, OccupancyGrid, Odometry


class PathPlanningNode(AbstractNode):

    def initialization(self):
        self._frame = Frame()
        self.planners = {
            Frame.TASK_TYPE_INIT: IdlePlanner(self),
            Frame.TASK_TYPE_PLAN_TO_GOAL: APathPlanner(self, self._frame),
            Frame.TASK_TYPE_COV_PLAN: CoveragePlanner(self, self._frame)
        }

        self._route_publisher = rospy.Publisher(rospy.get_param('/planner/topics/route/control'),
                                                Route,
                                                queue_size=10)
                                    
        # self._route_publisher = rospy.Publisher(rospy.get_param('/planner/topics/route/path_cutter'),
        #                                         Route,
        #                                         queue_size=10)

        self._last_task_type = Frame.TASK_TYPE_INIT

        rospy.Subscriber(rospy.get_param('/planner/topics/task_polygon_planning'),
                         RouteTaskPolygon, 
                         self._frame.receive_task_polygon_planning)
        rospy.Subscriber(rospy.get_param('/planner/topics/task_to_point_planning'),
                         RouteTaskToPoint, 
                         self._frame.receive_task_to_point_planning)
        rospy.Subscriber(rospy.get_param('/planner/topics/costmap'),
                         OccupancyGrid, self._frame.receive_occupancy_grid_map)
        rospy.Subscriber(rospy.get_param('/planner/topics/localization'), Localization,
                         self._frame.receive_localization)

    def publish_route(self, message):
        self._route_publisher.publish(message)

    def work(self):
        with self._frame.lock():
            if self._last_task_type != self._frame.get_current_task_type():
                self.planners[self._last_task_type].reset()
                self._last_task_type = self._frame.get_current_task_type()
                self.planners[self._last_task_type].initialize()
            
            self.planners[self._last_task_type].execute()


class IdlePlanner(object):
    def __init__(self, node):
        self._node = node

    def reset(self):
        pass

    def initialize(self):
        pass

    def execute(self):
        route = Route()
        route.header.stamp = rospy.get_rostime()
        self._node.publish_route(route)
    
if __name__ == '__main__':
    PathPlanningNode('PathPlanningNode', 5).run()