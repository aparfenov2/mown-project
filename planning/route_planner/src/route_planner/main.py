
#!/usr/bin/env python3
import rospy
import numpy as np

from abstractnode import AbstractNode
from continious_state_planner import AstarPathPlanner, GridGenerator, AStar
from coverage_path_planner import CoveragePathClient
from enginx_msgs.msg import Localization, PointWithSpeed, Route, RouteTaskPolygon, RouteTaskToPoint, ProgressRoutePlanner
from geometry_msgs.msg import Point32, PoseStamped
from nav_msgs.msg import Path, OccupancyGrid
from tf.transformations import euler_from_quaternion


class RoutePlannerNode(AbstractNode):
    IDLE = 'idle'
    COVER = "cover"
    PATH = "to_point"

    MAX_SPEED = 0.10

    def initialization(self):
        self.__astar_planner = AstarWrapper()
        # self.__coverage_path_client = CoveragePathClient()

        self.__task = None 
        self.__path = None
        self.__position = None

        self.__states_dict = {
            self.IDLE: self.__idle_process,
            self.PATH: self.__pose_task_process,
            # self.COVER: self.__polygon_task_progress
        }
        self.__state = self.IDLE

        self.obstacles = set()
        self.resolution = 1.0
 
        self.__route_publisher = rospy.Publisher('/planner/route', Route, queue_size=10)
        self.__progress_route_publisher = rospy.Publisher('/planner/route_progress', ProgressRoutePlanner, queue_size=10)

        rospy.Subscriber('/planner/route_task_polygon', RouteTaskPolygon, self.__task_polygon_callback)
        rospy.Subscriber('/planner/route_task_to_point', RouteTaskToPoint, self.__task_to_point_callback)
        rospy.Subscriber('/planner/localization', Localization, self.__localization_callback)
        rospy.Subscriber('/planner/occupancy_grid_map', OccupancyGrid, self.__occupancy_grid_map_callback)
    
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

    def __task_polygon_callback(self, message):
        self.__task = message
        self.__state = self.COVER

    def __task_to_point_callback(self, message):
        self.__task = message
        self.__state = self.PATH

    def __occupancy_grid_map_callback(self, message):
        width = message.info.width
        height = message.info.height
        resolution = message.info.resolution

        self.__astar_planner.set_scale(resolution)

        for x in range(width):
            for y in range(height):
                if message.data[x + width * y] == 1:
                    pass

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

        path = self.__astar_planner.search(robot_pose, target_pose, obstacles=self.obstacles)

        route = Route()

        for point in path:
            pws = PointWithSpeed()
            pws.x = point[0]
            pws.y = point[1]
            pws.speed = self.MAX_SPEED
            route.route.append(pws)

        self.__route_publisher.publish(route)

        # if self.__distance(target_pose, robot_pose) < 1.0:
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


class AstarWrapper(object):
    def __init__(self, scale=0.1):
        self.scale = scale

    def set_scale(self, scale):
        self.scale = scale

    def search(self, source_point, goal_point, obstacles):
        grid_source = self.convert_to_grid(source_point)
        grid_goal = self.convert_to_grid(goal_point)
        # print("From: {}, To: {}".format(
        #     grid_source,
        #     grid_goal
        # ))

        astar = AStar(grid_source, grid_goal, "euclidean", obstacles=obstacles)
        path, visited = astar.searching()
        
        result = []
        for point in path:
            result.append(self.convert_to_coordinate(point))

        # print("GOT result, {} points".format(len(result)))
        result = reversed(result)
        return result

    def convert_to_grid(self, point):
        return tuple([ int(p / self.scale) for p in point[:2]])

    def convert_to_coordinate(self, point):
        return [ float(p * self.scale) for p in point]
