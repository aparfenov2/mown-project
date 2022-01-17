
#!/usr/bin/env python3
from time import sleep

from numpy.lib.function_base import append
import rospy
import numpy as np
from tf.transformations import euler_from_quaternion

from .astart_wrapper import AstarWrapper
from .continuos_astar_wrapper import ContinuosAstarWrapper
from abstractnode import AbstractNode
from continious_state_planner import AstarPathPlanner, GridGenerator, AStar
from coverage_path_planner import CoveragePathClient

from geometry_msgs.msg import Point32, PoseStamped
from nav_msgs.msg import Path, OccupancyGrid
from enginx_msgs.msg import (Localization, PointWithSpeed, Route, 
                             RouteTaskPolygon, RouteTaskToPoint, ProgressRoutePlanner)


class APathPlanner(object):
    IDLE = 'idle'
    COVER = "cover"
    PATH = "to_point"

    COV_STATE_TO_START = 'csts'
    COV_STATE_WALK = 'csw'

    MAX_SPEED = 0.10

    def __init__(self, node, frame):

        self.__astar_planner = ContinuosAstarWrapper(frame)
        self._frame = frame
        self._node = node

        self._stop_planning_threshold = 0.5

        # self.__states_dict = {
        #     self.IDLE: self.__idle_process,
        #     self.PATH: self.__pose_task_process,
        #     self.COVER: self.__polygon_task_progress
        # }
        self.__state = self.IDLE

        self.obstacles = set()
        self.resolution = 1.0

        self.distance_threshold = 1.0
 
        self.__route_publisher = rospy.Publisher(
            rospy.get_param('/planner/topics/route/path_planner'), 
            Route, 
            queue_size=10
        )
        self.__progress_route_publisher = rospy.Publisher(rospy.get_param('/planner/topics/path_progress'), ProgressRoutePlanner, queue_size=10)

    def reset(self):
        pass

    def initialize(self):
        pass
    
    def execute(self):
        self.__pose_task_process()

    # def __task_to_point_callback(self, message):
    #     print('GOT', message)
    #     self.__task = message
    #     self.__state = self.PATH if message.header.stamp.to_sec() != 0 else self.IDLE

    # def __occupancy_grid_map_callback(self, message):
    #     width = message.info.width
    #     height = message.info.height
    #     resolution = message.info.resolution

    #     self.obstacles = set()

        # print('Grid map info:', message.info, set(message.data))

        # self.__astar_planner.set_scale(resolution)

        # for x in range(width):
        #     for y in range(height):
        #         if message.data[x + width * y] == 1:
        #             pass

    # def __localization_callback(self, message):
    #     self.__position = message

    # def __publish_route(self, message):
    #     u"""
    #     Args:
    #         message(Route).
    #     """
    #     self.__route_publisher.publish(message)

    # def __idle_process(self):
    #     route = Route()
    #     self.__route_publisher.publish(route)

    def __calc_astar_path_to_target(self, target):
        
        loca = self._frame.get_localization()
        robot_pose = self.__localization2tuple(loca)

        path = self.__astar_planner.search(robot_pose, target)
        return path

    def __pose_task_process(self):
        if self._frame.get_localization() is None:
            return

        target_pose = self._frame.get_path_task().target_pose
        target_pose = self.__pose2tuple(target_pose)

        loca = self._frame.get_localization()
        robot_pose = self.__localization2tuple(loca)

        if self.__distance(target_pose, robot_pose) < self._stop_planning_threshold:
            return

        path = self.__calc_astar_path_to_target(target_pose)

        route = Route()

        for point in path:
            pws = PointWithSpeed()
            pws.x = point[0]
            pws.y = point[1]
            pws.speed = self.MAX_SPEED
            route.route.append(pws)

        self._node.publish_route(route)

        self.__send_task_done()

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

    def publish_path(self, points):
        # path_pub = rospy.Publisher('/planner/path_test', Path, queue_size=10)
        path = Path()
        path.header.stamp = rospy.get_rostime()
        path.header.frame_id = 'base_link'
        for point in points:
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = point.x
            pose.pose.position.y = point.y
            path.poses.append(pose)
        self.path_pub.publish(path)
