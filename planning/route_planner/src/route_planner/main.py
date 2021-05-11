import rospy
import numpy as np

from abstractnode import AbstractNode
from continious_state_planner import AstarPathPlanner
from coverage_path_planner import CoveragePathClient
from enginx_msgs.msg import Localization, PointWithSpeed, Route, RouteTask
from geometry_msgs.msg import Point32, PoseStamped
from nav_msgs.msg import Path


class RoutePlannerNode(AbstractNode):
    def initialization(self):
        self.__astar_planner = AstarPathPlanner()
        self.__coverage_path_client = CoveragePathClient()

        self.__task = None 
        self.__path = None
        self.__position = None

        self.__states_dict = {
            "idle": self.__idle_process,
            RouteTask.GO_TO_POSE_TASK: self.__pose_task_process,
            RouteTask.EXPLORE_POLYGON_TASK: self.__polygon_task_progress
        }
        self.__state = "idle"

        self.__route_publisher = rospy.Publisher('/route', Route, queue_size=10)

        rospy.Subscriber('/route_task', RouteTask, self.__task_callback)
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

    def __set_state(self, new_state):
        if new_state not in self.__states_dict.keys():
            err_msg = "Got wrong state name: {0}".format(new_state)
            rospy.logerr(err_msg)
            raise AttributeError(err_msg)

        self.__state = new_state

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
        self.__set_state(message.task_type)

    def __localization_callback(self, message):
        self.__position = message

    def __publish_route(self, message):
        u"""
        Args:
            message(Route).
        """
        self.__route_publisher.publish(message)

    def __idle_process(self):
        pass
    
    def __pose_task_process(self):
        target_pose = self.task.target_pose
        target_pose = self.__pose2numpyarray(target_pose)

        robot_pose = self.__localization2numpyarray(self.__position)

        path = self.__astar_planner(robot_pose, target_pose)

        route = Route()

        for point in path:
            pws = PointWithSpeed()
            pws.x = point[0]
            pws.y = point[1]
            route.route.append(pws)

        self.publish(route)

        if self.__distance(target_pose, robot_pose) < 2.0:
            self.__send_task_done()
            self.__set_state('idle')

    def __polygon_task_progress(self):
        
        source = self.__position2point32(self.__position)
        path = self.__coverage_path_client.get_path(self.__task.target_polygon, source).path

        message = self.__point32array2route(path)
        self.__publish_route(message)

        if self.__some_conditions():
            self.__send_task_done()
            self.__set_state('idle')



class TargetPoseTaskProcessor(object):
    def __init__(self, node):
        self.__node = node

    def process(self):
        if target