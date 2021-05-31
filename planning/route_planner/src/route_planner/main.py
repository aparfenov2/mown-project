
#!/usr/bin/env python3
import rospy
import numpy as np

from abstractnode import AbstractNode
from continious_state_planner import AstarPathPlanner
from coverage_path_planner import CoveragePathClient
from enginx_msgs.msg import Localization, PointWithSpeed, Route, RouteTask, ProgressRoutePlanner
from geometry_msgs.msg import Point32, PoseStamped
from nav_msgs.msg import Path
from tf.transformations import euler_from_quaternion


class RoutePlannerNode(AbstractNode):
    IDLE = 'idle'
    COVER = RouteTask.EXPLORE_POLYGON_TASK
    PATH = RouteTask.GO_TO_POSE_TASK

    MAX_SPEED = 5.0

    def initialization(self):
        self.__astar_planner = AstarPathPlanner()
        self.__coverage_path_client = CoveragePathClient()

        self.__task = None 
        self.__path = None
        self.__position = None

        self.__states_dict = {
            self.IDLE: self.__idle_process,
            self.PATH: self.__pose_task_process,
            self.COVER: self.__polygon_task_progress
        }
        self.__state = self.IDLE
 
        self.__route_publisher = rospy.Publisher('/route', Route, queue_size=10)
        self.__progress_route_publisher = rospy.Publisher('/route_progress', ProgressRoutePlanner, queue_size=10)

        rospy.Subscriber('/route_task', RouteTask, self.__task_callback)
        rospy.Subscriber('/localization', Localization, self.__localization_callback)
        print('HERE')
    
    def work(self):
        self.__states_dict[self.__state]()        

    def __check_state(self, new_state):
        if new_state not in self.__states_dict.keys():
            err_msg = "Got wrong state name: {0}, EXIST {1}".format(new_state, self.__states_dict.keys())
            rospy.logerr(err_msg)
            raise AttributeError(err_msg)

    def __point32array2route(self, path):
        """
        Args:
            path (Point32[]).
        
        Return:
            Route.
        """
        route = Route()
        route.header.stamp = rospy.get_rostime()
        route.path = [PointWithSpeed(x=point.x, y=point.y, speed=self.MAX_SPEED) for point in path]
        return route

    def __task_callback(self, message):
        self.__task = message
        self.__check_state(message.task_type)
        self.__state = message.task_type

    def __localization_callback(self, message):
        self.__position = message

    def __publish_route(self, message):
        u"""
        Args:
            message(Route).
        """
        self.__route_publisher.publish(message)

    def __idle_process(self):
        route = Route()
        self.__route_publisher.publish(route)
    
    def __pose_task_process(self):
        if self.__position is None:
            return

        target_pose = self.__task.target_pose
        target_pose = self.__pose2tuple(target_pose)

        robot_pose = self.__localization2tuple(self.__position)

        path = self.__astar_planner.astar_statespace(robot_pose, target_pose)

        route = Route()

        for point in path:
            pws = PointWithSpeed()
            pws.x = point[0]
            pws.y = point[1]
            pws.speed = self.MAX_SPEED
            route.route.append(pws)

        self.__route_publisher.publish(route)

        if self.__distance(target_pose, robot_pose) < 1.0:
            self.__send_task_done()
            self.__switch_to_idle()

    def __pose2tuple(self, pose):
        q = pose.orientation
        pose = pose.position
        yaw = euler_from_quaternion(np.array([
            q.x, q.y, q.z, q.w
        ]))[2]
        pos_numpy = (pose.x, pose.y, yaw)
        return pos_numpy

    def __localization2tuple(self, pos):
        pos = (pos.pose.x, pos.pose.y, pos.yaw)
        return pos

    def __distance(self, p1, p2):
        p1 = np.array(p1)
        p2 = np.array(p2)
        return np.linalg.norm(p1 - p2)
    
    def __switch_to_idle(self):
        self.__path = None
        self.__state == self.IDLE

    def __send_task_done(self):
        progress = ProgressRoutePlanner()
        self.__progress_route_publisher.publish(progress)

    def __position2point32(self, pos):
        pos32 = Point32()
        pos32.x = pos.pose.x
        pos32.y = pos.pose.y
        return pos32

    def __polygon_task_progress(self):
        if self.__position is None:
            return

        if not self.__path:
            source = self.__position2point32(self.__position)
            self.__path = self.__coverage_path_client.get_path(self.__task.target_polygon, source).path

            message = self.__point32array2route(self.__path)
            self.__publish_route(message)
        
        if self.__stop_condition():
            self.__send_task_done()
            self.__switch_to_idle()

    def __stop_condition(self):
        pos = self.__position

        # todo: find last available point
        last_point = self.__path.route[-1]

        pos = np.array([pos.pose.x, pos.pose.y])
        last_point = np.array([last_point.x, last_point.y])

        return np.linalg.norm(pos - last_point) < 1.0
