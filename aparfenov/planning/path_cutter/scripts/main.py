#!/usr/bin/env python3
from time import sleep
from numpy.lib.function_base import append
import rospy
import numpy as np
import threading

from abstractnode import AbstractNode
from continious_state_planner import AstarPathPlanner, GridGenerator, AStar
from coverage_path_planner import CoveragePathClient
from tf.transformations import euler_from_quaternion

from geometry_msgs.msg import Point32, PoseStamped
from nav_msgs.msg import Path, OccupancyGrid, Odometry
from enginx_msgs.msg import Localization, PointWithSpeed, Route, RouteTaskPolygon, RouteTaskToPoint, ProgressRoutePlanner


def convert_to_grid(scale, point):
    return tuple([ int(p / scale) for p in point[:2] ])


def convert_to_coordinate(scale, point):
    return [ float((p + 0.5) * scale) for p in point] 


class PathCutter(AbstractNode):

    def initialization(self):
        self.__position = None
        self.__obstacles = set()
        self.__robot_radius = rospy.get_param('/planner/path_cutter/robot_radius')

        self.__rlock = threading.RLock()

        self.__route_publisher = rospy.Publisher(
            rospy.get_param('/planner/topics/route/control'),
            Route, queue_size=10
        )
        
        rospy.Subscriber(rospy.get_param('/planner/topics/route/path_cutter'),
                         Route, self.__task_route_callback)
        rospy.Subscriber(rospy.get_param('/planner/topics/localization'), Localization,
                         self.__localization_callback)
        rospy.Subscriber(rospy.get_param('/planner/topics/costmap'),
                         OccupancyGrid, self.__occupancy_grid_map_callback)

    def work(self):
        pass

    def __task_route_callback(self, message):
        with self.__rlock:
            if len(message.route) == 0 or len(self.__obstacles) == 0 or self.__position is None:
                self.__publish_route(message)
                return

            path = []
            for point in message.route:
                path.append([point.x, point.y])

            np_path = np.array(path)
            ego_pos = np.array(self.__position)

            min_index = np.argmin(np.linalg.norm(np_path - ego_pos))

            if min_index == len(path) - 1:
                self.__publish_route(message)
                return

            stop_index = self.__check_path_for_collition(path)

            if stop_index is None:
                stop_index = len(message.route[:stop_index])
            else:
                stop_index += len(path) + 1

            message.route = message.route[:stop_index]

            self.__publish_route(message)

    def __check_path_for_collition(self, path):
        stop_index = None

        for i, point in enumerate(path):
            x, y = self.__convert_to_grid_map(point)
            radius = int(self.__robot_radius / self.__scale)
            rob_set = BresenhamCircle(radius, x, y)
            collision = rob_set & self.__obstacles
            if len(collision) > 0:
                stop_index = i
                
        return stop_index

    def __convert_to_grid_map(self, point):
        return convert_to_grid(self.__scale, point)

    def __occupancy_grid_map_callback(self, message):
        with self.__rlock:
            width = message.info.width
            height = message.info.height
            resolution = message.info.resolution

            x0, y0 = convert_to_grid(
                resolution,
                [message.info.origin.position.x, message.info.origin.position.y]
            )

            self.__scale = resolution
            
            self.__obstacles = set()

            for y in range(height):
                for x in range(width):
                    if message.data[y + width * x] > 0:
                        self.__obstacles.add((
                            x + x0,
                            y + y0
                        ))

    def __localization_callback(self, message):
        with self.__rlock:
            self.__position = [
                message.pose.x,
                message.pose.y
            ]

    def __publish_route(self, message):
        self.__route_publisher.publish(message)


class BresenhamCircle(set):
    def __init__(self, radius, x0=0, y0=0):
        super(BresenhamCircle, self).__init__()
        points = self.calc_points(radius, x0, y0)
        self.update(points)

    def tolist(self):
        return list(self)

    def calc_points(self, radius, x0, y0):
        switch = 3 - 2 * radius
        points = []
        x = 0
        y = radius
        while x <= y:
            self.draw_line(points, -y + x0, x + y0, y + x0, x + y0)
            self.draw_line(points, -x + x0, y + y0, x + x0, y + y0)
            self.draw_line(points, -x + x0, -y + y0, x + x0, -y + y0)
            self.draw_line(points, -y + x0, -x + y0, y + x0, -x + y0)
            if switch < 0:
                switch = switch + 4 * x + 6
            else:
                switch = switch + 4 * (x - y) + 10
                y = y - 1
            x = x + 1

        return points

    def draw_line(self, points, x0, y0, x1, y1):
        if x1 == x0 and y1 == y0:
            points.append((x1, y1))
            return

        dx = x1 - x0
        dy = y1 - y0

        xsign = 1 if dx > 0 else -1
        ysign = 1 if dy > 0 else -1

        dx = int(abs(dx))
        dy = int(abs(dy))

        if dx > dy:
            xx, xy, yx, yy = xsign, 0, 0, ysign
        else:
            dx, dy = dy, dx
            xx, xy, yx, yy = 0, ysign, xsign, 0

        D = 2*dy - dx
        y = 0

        for x in range(dx + 1):
            points.append((x0 + x*xx + y*yx, y0 + x*xy + y*yy))
            if D >= 0:
                y += 1
                D -= 2*dx
            D += 2*dy


if __name__ == '__main__':
    PathCutter('PathCutter', 5).run()