
#!/usr/bin/env python3
from time import sleep
from numpy.lib.function_base import append
import rospy
import numpy as np

from abstractnode import AbstractNode
from continious_state_planner import AstarPathPlanner, GridGenerator, AStar
from coverage_path_planner import CoveragePathClient
from enginx_msgs.msg import Localization, PointWithSpeed, Route, RouteTaskPolygon, RouteTaskToPoint, ProgressRoutePlanner
from geometry_msgs.msg import Point32, PoseStamped
from nav_msgs.msg import Path, OccupancyGrid
from tf.transformations import euler_from_quaternion

from nav_msgs.msg import Path
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped


class RoutePlannerNode(AbstractNode):
    IDLE = 'idle'
    COVER = "cover"
    PATH = "to_point"

    COV_STATE_TO_START = 'csts'
    COV_STATE_WALK = 'csw'

    MAX_SPEED = 0.10

    def initialization(self):
        self.__astar_planner = AstarWrapper()
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
        self.__cov_state = None

        self.obstacles = set()
        self.resolution = 1.0

        self.distance_threshold = 1.0
 
        self.__route_publisher = rospy.Publisher(
            rospy.get_param('/planner/topics/route/path_planner'), 
            Route, 
            queue_size=10
        )
        self.__progress_route_publisher = rospy.Publisher(rospy.get_param('/planner/topics/path_progress'), ProgressRoutePlanner, queue_size=10)

        self.path_pub = rospy.Publisher('/planner/path_test', Path, queue_size=10)
        # rospy.Subscriber('/planner/route_task_polygon', RouteTaskPolygon, self.__task_polygon_callback)
        rospy.Subscriber(
            rospy.get_param('/planner/topics/task_to_point_planning') + '/RoutePlannerNode', 
            RouteTaskToPoint, 
            self.__task_to_point_callback
        )
        rospy.Subscriber(
            rospy.get_param('/planner/topics/localization'),
            Localization, 
            self.__localization_callback
            )
        rospy.Subscriber(
            rospy.get_param('/planner/topics/costmap'),
            OccupancyGrid, 
            self.__occupancy_grid_map_callback
        )
    
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
        route.route = [PointWithSpeed(x=point.x, y=point.y, speed=self.MAX_SPEED) for point in path]
        return route

    def __task_polygon_callback(self, message):
        self.__task = message
        self.__state = self.COVER

    def __task_to_point_callback(self, message):
        print('GOT', message)
        self.__task = message
        self.__state = self.PATH if message.header.stamp.to_sec() != 0 else self.IDLE

    def __occupancy_grid_map_callback(self, message):
        width = message.info.width
        height = message.info.height
        resolution = message.info.resolution

        self.obstacles = set()

        # print('Grid map info:', message.info, set(message.data))

        # self.__astar_planner.set_scale(resolution)

        # for x in range(width):
        #     for y in range(height):
        #         if message.data[x + width * y] == 1:
        #             pass

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

    def __calc_astar_path_to_target(self, target):

        robot_pose = self.__localization2tuple(self.__position)

        path = self.__astar_planner.search(robot_pose, target, obstacles=self.obstacles)
        return path

    def __pose_task_process(self):
        if self.__position is None:
            return

        target_pose = self.__task.target_pose
        target_pose = self.__pose2tuple(target_pose)

        # robot_pose = self.__localization2tuple(self.__position)

        path = self.__calc_astar_path_to_target(target_pose)

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
        return
        # if self.__position is None:
        #     return

        # if not self.__path:
        #     source = self.__position2point32(self.__position)
        #     self.__path = self.__coverage_path_client.get_path(self.__task.target_polygon, source).path

        #     if self.__check_pos_and_first_point_len(self.__path):
        #         # self.__path = self.__add_path_to_start(self.__path)
        #         self.__cov_state = self.COV_STATE_TO_START
        #     else:
        #         self.__cov_state = self.COV_STATE_WALK


        #     message = self.__point32array2route(self.__path)
        #     self.__publish_route(message)

        #     self.publish_path(self.__path)
        
        # if self.__stop_condition():
        #     self.__send_task_done()
        #     self.__switch_to_idle()

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
        sleep(1.0)
        self.path_pub.publish(path)

    def __check_pos_and_first_point_len(self, path):
        u"""
        Args:
            path (Point32[]).

        Return:
            bool.
        """
        if len(path) < 2:
            return True

        first_point = np.array((path[0].x, path[0].y))
        current_pos = np.array((self.__position.pose.x, self.__position.pose.y))

        return np.linalg.norm(first_point - current_pos) > self.distance_threshold

    def __add_path_to_start(self, path):
        target = (path[0].x, path[0].y)
        
        path_to_target = self.__calc_astar_path_to_target(target)

        points_to_start = list()
        for p in path_to_target:
            points_to_start.append(Point32(
                x=p[0],
                y=p[1]
            ))

        path = points_to_start + path
        return path

    def __stop_condition(self):
        pos = self.__position

        # todo: find last available point
        last_point = self.__path[-1]

        pos = np.array([pos.pose.x, pos.pose.y])
        last_point = np.array([last_point.x, last_point.y])

        return np.linalg.norm(pos - last_point) < 1.0


# class CoverageStateMachine(object):
#     def __init__(self, node):
#         self.node = node

#         self.cur_processor = None

#     def process(self):
#         if self.state == None:
#             self.__path = None
#             self.state = 

    # def first_processor(self):


    # def idle(self):
    #     pass

    # def go_to_start(self):
    #     pass

    # def walk(self):
    #     pass

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
        result = self.smooth_path(np.array(list(result)), 0.5, 0.2, 0.001)
        return result.tolist()

    def smooth_path(self, path, alpha, betta, tol):
        npath = np.copy(path)

        npoints = npath.shape[0]

        change = tol
        while change >= tol:
            change = 0.0
            for i in range(1, npoints - 1):
                y_saved = np.copy(npath[i])

                npath[i] += alpha * (path[i] - npath[i]) + betta * (npath[i - 1] + npath[i + 1] - 2 * (npath[i]))

                change += abs(np.linalg.norm(y_saved - npath[i]))

        return npath

    def convert_to_grid(self, point):
        return tuple([ int(p / self.scale) for p in point[:2]])

    def convert_to_coordinate(self, point):
        return [ float(p * self.scale) for p in point]
