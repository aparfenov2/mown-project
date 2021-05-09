import rospy
import numpy as np

from abstractnode import AbstractNode
from coverage_path_planner import CoveragePathClient
from enjnx_msgs.msgs import GlobalTask, Localization, Route
from geometry_msgs.msgs import Point32, PoseStamped
from nav_msgs.msgs import Path


class RoutePlannerNode(AbstractNode):
    def initialization(self):
        self.__coverage_path_client = CoveragePathClient()
        self.__task = None 
        self.__path = None
        self.__position = None

        self.__route_publisher = rospy.Publisher('/route_path', Route, queue_size=10)

        rospy.Subscriber('/route_task', GlobalTask, self.__task_callback)
        rospy.Subscriber('/localization', Localization, self.__localization_callback)
    
    def work(self):
        if self.__task is None or self.__position is None:
            return

        # if self.__path is None:
        source = self.__position2point32(self.__position)
        path = self.__coverage_path_client.get_path(self.__task.polygon, source).path

        message = self.__point32array2route(path)
        self.__publish_route(message)

        self.__task = None

    def __point32array2route(self, position):
        u"""
        Args:
            position (Localization).
        
        Return:
            Point32.
        """
        point32 = Point32(x=position.x, y=position.y)
        return point32

    def __point32array2path(self, path):
        """
        Args:
            path (Point32[]).
        
        Return:
            Route.
        """
        route = Route()
        route.header.stamp = rospy.get_rostime()
        route.path = path
        return route

    def __task_callback(self, message):
        self.__task = message

    def __localization_callback(self, message):
        self.__position = message

    def __publish_route(self, message):
        u"""
        Args:
            message(Route).
        """
        self.__route_publisher.publish(message)
