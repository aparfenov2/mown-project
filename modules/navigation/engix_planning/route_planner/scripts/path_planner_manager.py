#!/usr/bin/env python3

from time import sleep
from numpy.lib.function_base import append
from genpy import message
import rospy
import numpy as np

from abstractnode import AbstractNode
from continious_state_planner import AstarPathPlanner, GridGenerator, AStar
from coverage_path_planner import CoveragePathClient
from engix_msgs.msg import Localization, PointWithSpeed, Route, RouteTaskPolygon, RouteTaskToPoint, ProgressRoutePlanner
from geometry_msgs.msg import Point32, PoseStamped
from nav_msgs.msg import Path, OccupancyGrid, Odometry
from tf.transformations import euler_from_quaternion

# from nav_msgs.msg import Path
# from nav_msgs.msg import Odometry
# from geometry_msgs.msg import PoseStamped


class PathPlannerManager(AbstractNode):

    def initialization(self):

        self.planners = {
            "idle": IdlePlanner(self, enable_flag=True),
            "task_move_to_point": PathToPointPlanner(self, False),
            "task_do_coverage_planning": CoveragePathInPolygon(self, False)
        }

        self.route_publisher = rospy.Publisher(rospy.get_param('/planner/topics/route/path_cutter'), Route, queue_size=10)

        rospy.Subscriber(rospy.get_param('/planner/topics/task_polygon_planning'),
                         RouteTaskPolygon, 
                         self.__task_polygon_callback)

        rospy.Subscriber(rospy.get_param('/planner/topics/task_to_point_planning'),
                         RouteTaskToPoint, 
                         self.__task_to_point_callback)

    def work(self):
        pass

    def __task_polygon_callback(self, message):
        # self.__task = message
        # self.__state.status = self.__state.PROCESS

        for key, value in self.planners.items():
            if key != 'task_do_coverage_planning' and value.enable_flag:
                value.disable()

        self.planners['task_do_coverage_planning'].enable()
        self.planners['task_do_coverage_planning'].send_plan_to_planner(message)

    def __task_to_point_callback(self, message):
        # self.__task = message
        # self.__state.status = self.__state.IDLE

        for key, value in self.planners.items():
            if key != 'task_move_to_point' and value.enable_flag:
                value.disable()

        self.planners['task_move_to_point'].enable()
        self.planners['task_move_to_point'].send_plan_to_planner(message)


class CommonPlanner(object):
    def __init__(self, node, enable_flag=True):
        self.node = node
        self.enable_flag = enable_flag
    
    def enable(self):
        self.enable_flag = True

    def disable(self):
        self.enable_flag = False


class IdlePlanner(CommonPlanner):
    pass


class PathToPointPlanner(CommonPlanner):
    def __init__(self, *args, **kwargs):
        super(PathToPointPlanner, self).__init__(*args, **kwargs)
        self.sender = rospy.Publisher(rospy.get_param('/planner/topics/task_to_point_planning') + '/RoutePlannerNode', RouteTaskToPoint, queue_size=10)
        rospy.Subscriber(rospy.get_param('/planner/topics/route/path_planner'), Route, self.callback)

    def send_plan_to_planner(self, message):
        self.sender.publish(message)

    def disable(self):
        super(PathToPointPlanner, self).disable()
        message = RouteTaskToPoint()
        self.sender.publish(message)

    def callback(self, message):
        if self.enable_flag:
            self.node.route_publisher.publish(message)


class CoveragePathInPolygon(CommonPlanner):
    def __init__(self, *args, **kwargs):
        super(CoveragePathInPolygon, self).__init__(*args, **kwargs)
        self.sender = rospy.Publisher(rospy.get_param('/planner/topics/task_polygon_planning') + '/CoveragePlannerNode', RouteTaskPolygon, queue_size=10)
        rospy.Subscriber(rospy.get_param('/planner/topics/route/coverage_planner'), Route, self.callback)

    def send_plan_to_planner(self, message):
        self.sender.publish(message)

    def disable(self):
        super(CoveragePathInPolygon, self).disable()
        message = RouteTaskPolygon()
        message.header.stamp = rospy.get_rostime()
        self.sender.publish(message)

    def callback(self, message):
        if self.enable_flag:
            self.node.route_publisher.publish(message)


if __name__ == '__main__':
    PathPlannerManager('PathPlannerManager', 5).run()
