from cProfile import label
from math import fabs
from readline import parse_and_bind
from select import select
from attr import s
from scipy import interpolate
from time import time
import itertools

import casadi
import dubins
import rospy
import numpy as np
from scipy.optimize import NonlinearConstraint
from scipy import optimize
from behavior_planning.algorithms import ContinuosAstarWrapper
from task_behavior_engine.tree import Node, NodeStatus

from .trajectory_smoother import TrajectorySmoother
from coverage_path_planner import CoveragePathClient
from .speed_generator import SpeedGenerator

from nav_msgs.msg import Path
from geometry_msgs.msg import Point, Point32, PoseStamped
from engix_msgs.msg import Route, PointWithSpeed, SpeedGeneratorDebug


def find_nearest_index(path, current_pose):
    if len(path) == 1:
        return 0

    path = np.array(path)
    cur_pose = np.array(current_pose)

    idx = (np.linalg.norm(path - cur_pose, axis=1)).argmin()
    return idx


class CoveragePath:
    def __init__(self, path, expand_len, target_speed) -> None:
        self._last_nearest_index = 0
        self._path = path
        self._expand_len = expand_len
        self._speed_profile = []
        self._generate_speed_profile(target_speed)

    def _generate_speed_profile(self, target_speed):
        self._speed_profile = [target_speed] * len(self._path)

        for i in range(len(self._speed_profile)):
            self._speed_profile[i] = min((i + 1) * 0.2, self._speed_profile[i])

        for i in range(1, len(self._speed_profile)):
            self._speed_profile[-i] = min((i-1) * 0.3, self._speed_profile[-i])

    def update(self, robot_position):
        first_p = max(0, self._last_nearest_index - self._expand_len)
        second_p = min(len(self._path) - 1, self._last_nearest_index + self._expand_len) + 1
        temp_sub_path = self._path[first_p: second_p]
        nearest_ind = find_nearest_index(temp_sub_path, robot_position)

        self._last_nearest_index = first_p + nearest_ind

    def sub_path(self) -> list:
        first_p = max(0, self._last_nearest_index - self._expand_len)
        second_p = min(len(self._path) - 1, self._last_nearest_index + self._expand_len) + 1
        sub_path = self._path[first_p: second_p]
        return sub_path

    def sub_speed_profile(self):
        first_p = max(0, self._last_nearest_index - self._expand_len)
        second_p = min(len(self._speed_profile) - 1, self._last_nearest_index + self._expand_len) + 1
        sub_profile = self._speed_profile[first_p: second_p]
        return sub_profile

    def to_path_message(self):
        path = Path()
        path.header.stamp = rospy.get_rostime()
        path.header.frame_id = 'world'

        for x, y in self._path:
            pose_stamped = PoseStamped()
            pose_stamped.header = path.header
            pose_stamped.pose.position.x = x
            pose_stamped.pose.position.y = y

            path.poses.append(pose_stamped)
        return path


class CoverageNode(Node):
    def __init__(self, name, frame, params, *args, **kwargs):
        super(CoverageNode, self).__init__(name=name,
                                           run_cb=self.run,
                                           *args, **kwargs)
        self._frame = frame
        self._smoother = TrajectorySmoother([10.0, 10.0, 10.1, 10.1])
        self._coverage_path = None
        self._target_speed = params['target_speed']
        self._turning_radius = params['dubins']['turning_radius']
        self._step_size = params['dubins']['step_size']

    def run(self, nodedata):
        # if self._is_path_sent():
        #     return NodeStatus(NodeStatus.SUCCESS)

        if self._is_new_task():
            path = self._get_coverage_path()
            expanded_path = self._expand_path(path)
            path_to_start = self._path_to_start(expanded_path)
            # cropped_path = self._crop_path(expanded_path)
            merged_path = self._merge_path(path_to_start, expanded_path)

            self._save_full_path(merged_path)
            # smooth_path = self._smooth_path(expanded_path)
            # target_path = self._create_message(path_to_start, smooth_path)
            # self._frame.set_trajectory(target_path)

            self._set_task_processed()
            self._last_nearest_index = 0
            # self._set_path_sent_flag()

        path, sub_speed_profile = self._find_sub_path()
        smooth_path = self._smooth_path(path)
        target_path = self._create_Task_message(smooth_path, sub_speed_profile)
        self._frame.set_trajectory(target_path)
        self._fill_debug()
        return NodeStatus(NodeStatus.SUCCESS)

    def _fill_debug(self):
        self._frame.debug_data.coverage_path = self._coverage_path.to_path_message()

    def _is_new_task(self):
        return self._frame.get_cov_task()

    def _set_task_processed(self):
        return self._frame.set_cov_task(False)

    def _merge_path(self, path_1: list, path_2: list) -> list:
        return path_1 + path_2

    def _save_full_path(self, new_coverage_path):
        self._coverage_path = CoveragePath(new_coverage_path, 30, self._target_speed)

    def _find_sub_path(self) -> list:
        localization_message = self._frame.get_localization()
        robot_pose = self.__localization2tuple(localization_message)
        self._coverage_path.update(robot_pose[:2])
        sub_path = self._coverage_path.sub_path()
        sub_speed_profile = self._coverage_path.sub_speed_profile()
        return sub_path, sub_speed_profile

    def _path_to_start(self, path):
        localization_message = self._frame.get_localization()
        robot_pose = self.__localization2tuple(localization_message)
        start_pose = (
            path[0][0],
            path[0][1],
            np.arctan2(path[1][1] - path[0][1], path[1][0] - path[0][0])
        )
        turning_radius = self._turning_radius
        step_size = self._step_size
        path = dubins.shortest_path(robot_pose, start_pose, turning_radius)
        configurations, _ = path.sample_many(step_size)

        path_to_start = []

        for point in configurations:
            path_to_start.append([point[0], point[1]])

        # for i in range(len(configurations)):
        #     path_to_start[i][0] = configurations[i][0]
        #     path_to_start[i][1] = configurations[i][1]

        # rospy.logwarn(f"{robot_pose=}, {start_pose=}, {path_to_start=}")

        return path_to_start

    def _is_path_sent(self) -> bool:
        return self._frame.get_cov_path_done()

    def _set_path_sent_flag(self) -> bool:
        return self._frame.set_cov_path_done(True)

    def _get_coverage_path(self) -> list:
        message = self._frame.route_task_task_polygon_message
        nav_path = message.path
        path = [(pose.pose.position.x, pose.pose.position.y) for pose in nav_path.poses]
        return path

    def _expand_path(self, path: list) -> list:
        expanded_path = list()

        for i in range(1, len(path)):
            p1 = path[i - 1]
            p2 = path[i]

            dist = self.calc_distance_btwn_points(p1, p2)
            if dist > 1.0:
                parts = int(dist / 0.2)
                new_points = self._get_equidistant_points(
                    p1, p2, parts
                )
                # converted_points = self._convert_list_to_point3_array(
                #     new_points)
                expanded_path += new_points
            else:
                expanded_path += [p1, p2]

        return expanded_path

    def _get_equidistant_points(self, p1, p2, parts):
        return list(zip(np.linspace(p1[0], p2[0], parts+1),
                        np.linspace(p1[1], p2[1], parts+1)))

    def calc_distance_btwn_points(self, p1, p2):
        v1 = np.array(p1)
        v2 = np.array(p2)
        return np.linalg.norm(v1 - v2)

    def _convert_list_to_point3_array(self, path):
        new_path = list()

        for point in path:
            p = Point32(
                x=point[0],
                y=point[1]
            )
            new_path.append(p)
        return new_path

    def _crop_path(self, path: list) -> list:
        localization = self._frame.get_localization()
        current_pose = (localization.pose.x, localization.pose.y)
        nearest_index = self._get_nearest_index(path, current_pose)
        return path[nearest_index:]

    def _smooth_path(self, path):
        localization_message = self._frame.get_localization()
        robot_pose = self.__localization2tuple(localization_message)

        # save_data = {
        #     "robot_position": robot_pose,
        #     "path": path
        # }

        # import json
        # with open('/tmp/json_data.json', 'w') as outfile:
        #     json.dump(save_data, outfile)
        robot_pose = [path[0][0], path[0][1], np.arctan2(path[1][1] - path[0][1], path[1][0] - path[0][0])]

        trajectory = self._smoother.smooth(path, robot_pose)
        new_points = list()
        for i in range(len(trajectory['x'])):
            new_points.append([trajectory['x'][i], trajectory['y'][i]])
        return new_points

    def __localization2tuple(self, pos):
        pos = (pos.pose.x, pos.pose.y, pos.yaw)
        return pos

    def _get_nearest_index(self, path, current_pose):
        if len(path) == 1:
            return 0

        path = np.array(path)
        cur_pose = np.array(current_pose)

        idx = (np.linalg.norm(path - cur_pose, axis=1)).argmin()
        # path = np.array(self.__last_trajectory[idx:])

        # distances = np.linalg.norm(path - path[0], axis=1)
        # target_idx = distances.argmin()
        return idx

    def _create_Task_message(self, path, speed_profile):
        route = Route()
        route.header.stamp = rospy.get_rostime()

        for point, speed in zip(path, speed_profile):
            x, y = point
            pws = PointWithSpeed()
            pws.x = x
            pws.y = y
            pws.speed = speed
            pws.d_time = 0.0
            route.route.append(pws)

        return route

    def _create_message(self, path_to_start, path):
        route = Route()
        route.header.stamp = rospy.get_rostime()

        for x, y in itertools.chain(path_to_start, path[1:]):
            pws = PointWithSpeed()
            pws.x = x
            pws.y = y
            pws.speed = 0.5
            pws.d_time = 0.0
            route.route.append(pws)

        rospy.logwarn(f"{len(route.route)}, {len(path_to_start)}, {len(path)}")

        if len(route.route) > 0:
            path_points = min(len(route.route), 50)
            for i in range(1, path_points + 1):
                route.route[-i].speed = min(i * 0.05, route.route[-i].speed)

        return route


class CoveragePlanPathGeneratorNode(Node):
    def __init__(self, name, frame, *args, **kwargs):
        super(CoveragePlanPathGeneratorNode, self).__init__(name=name,
                                                            run_cb=self.run,
                                                            *args, **kwargs)
        self._frame = frame
        self._smoother = TrajectorySmoother([10.0, 10.0, 10.1, 10.1])
        self._coverage_path = None

        self._frame = frame
        self._last_stamp = 0.0

    def run(self, *args, **kwargs):
        if (not self._frame.localization.has_localization()):
            rospy.loginfo_throttle(1, "Waiting for localization.")
            return NodeStatus(NodeStatus.FAIL, "Frame doesn't have a localization")

        if (not self._frame.coverage_planning_task.has_message()):
            rospy.loginfo_throttle(1, "Waiting for CoveragePlanningTask message.")
            return NodeStatus(NodeStatus.FAIL, "Frame doesn't have a CoveragePlanningTask message")

        if self._is_new_task():
            path = self._frame.coverage_planning_task.path
            expanded_path = self._expand_path(path)
            path_to_start = self._path_to_start(expanded_path)
            # cropped_path = self._crop_path(expanded_path)
            merged_path = self._merge_path(path_to_start, expanded_path)

            self._save_full_path(merged_path)
            # smooth_path = self._smooth_path(expanded_path)
            # target_path = self._create_message(path_to_start, smooth_path)
            # self._frame.set_trajectory(target_path)

            self._set_task_processed()
            self._last_nearest_index = 0
            # self._set_path_sent_flag()
            self._last_stamp = self._frame.coverage_planning_task.stamp_sec

        path, sub_speed_profile = self._find_sub_path()
        smooth_path = self._smooth_path(path)
        target_path = self._create_Task_message(smooth_path, sub_speed_profile)
        self._frame.set_trajectory(target_path)
        self._fill_debug()
        return NodeStatus(NodeStatus.SUCCESS)

    def _fill_debug(self):
        self._frame.debug_data.coverage_path = self._coverage_path.to_path_message()

    def _is_new_task(self):
        cpt_stamp_time = self._frame.coverage_planning_task.stamp_sec
        ptt_stamp_time = self._frame.planning_task_type.stamp_sec
        return (self._last_stamp < cpt_stamp_time
                and cpt_stamp_time >= ptt_stamp_time)

    def _set_task_processed(self):
        return self._frame.set_cov_task(False)

    def _merge_path(self, path_1: list, path_2: list) -> list:
        return path_1 + path_2

    def _save_full_path(self, new_coverage_path):
        target_speed = self._frame.coverage_planning_task.target_speed
        self._coverage_path = CoveragePath(new_coverage_path, 30, target_speed)

    def _find_sub_path(self) -> list:
        robot_position = self._frame.localization.position
        # localization_message = self._frame.get_localization()
        # robot_pose = self.__localization2tuple(localization_message)
        self._coverage_path.update(robot_position)
        sub_path = self._coverage_path.sub_path()
        sub_speed_profile = self._coverage_path.sub_speed_profile()
        return sub_path, sub_speed_profile

    def _path_to_start(self, path):
        robot_pose = self._frame.localization.pose
        # robot_pose = self.__localization2tuple(localization_message)
        start_pose = (
            path[0][0],
            path[0][1],
            np.arctan2(path[1][1] - path[0][1], path[1][0] - path[0][0])
        )
        turning_radius = self._frame.coverage_planning_task.turning_radius
        step_size = self._frame.coverage_planning_task.step_size
        path = dubins.shortest_path(robot_pose, start_pose, turning_radius)
        configurations, _ = path.sample_many(step_size)

        path_to_start = []

        for point in configurations:
            path_to_start.append([point[0], point[1]])

        # for i in range(len(configurations)):
        #     path_to_start[i][0] = configurations[i][0]
        #     path_to_start[i][1] = configurations[i][1]

        # rospy.logwarn(f"{robot_pose=}, {start_pose=}, {path_to_start=}")

        return path_to_start

    def _is_path_sent(self) -> bool:
        return self._frame.get_cov_path_done()

    def _set_path_sent_flag(self) -> bool:
        return self._frame.set_cov_path_done(True)

    def _get_coverage_path(self) -> list:
        message = self._frame.route_task_task_polygon_message
        nav_path = message.path
        path = [(pose.pose.position.x, pose.pose.position.y) for pose in nav_path.poses]
        return path

    def _expand_path(self, path: list) -> list:
        expanded_path = list()

        for i in range(1, len(path)):
            p1 = path[i - 1]
            p2 = path[i]

            dist = self.calc_distance_btwn_points(p1, p2)
            if dist > 1.0:
                parts = int(dist / 0.2)
                new_points = self._get_equidistant_points(
                    p1, p2, parts
                )
                # converted_points = self._convert_list_to_point3_array(
                #     new_points)
                expanded_path += new_points
            else:
                expanded_path += [p1, p2]

        return expanded_path

    def _get_equidistant_points(self, p1, p2, parts):
        return list(zip(np.linspace(p1[0], p2[0], parts+1),
                        np.linspace(p1[1], p2[1], parts+1)))

    def calc_distance_btwn_points(self, p1, p2):
        v1 = np.array(p1)
        v2 = np.array(p2)
        return np.linalg.norm(v1 - v2)

    def _convert_list_to_point3_array(self, path):
        new_path = list()

        for point in path:
            p = Point32(
                x=point[0],
                y=point[1]
            )
            new_path.append(p)
        return new_path

    def _crop_path(self, path: list) -> list:
        localization = self._frame.get_localization()
        current_pose = (localization.pose.x, localization.pose.y)
        nearest_index = self._get_nearest_index(path, current_pose)
        return path[nearest_index:]

    def _smooth_path(self, path):
        localization_message = self._frame.get_localization()
        robot_pose = self.__localization2tuple(localization_message)

        # save_data = {
        #     "robot_position": robot_pose,
        #     "path": path
        # }

        # import json
        # with open('/tmp/json_data.json', 'w') as outfile:
        #     json.dump(save_data, outfile)
        robot_pose = [path[0][0], path[0][1], np.arctan2(path[1][1] - path[0][1], path[1][0] - path[0][0])]

        trajectory = self._smoother.smooth(path, robot_pose)
        new_points = list()
        for i in range(len(trajectory['x'])):
            new_points.append([trajectory['x'][i], trajectory['y'][i]])
        return new_points

    def __localization2tuple(self, pos):
        pos = (pos.pose.x, pos.pose.y, pos.yaw)
        return pos

    def _get_nearest_index(self, path, current_pose):
        if len(path) == 1:
            return 0

        path = np.array(path)
        cur_pose = np.array(current_pose)

        idx = (np.linalg.norm(path - cur_pose, axis=1)).argmin()
        # path = np.array(self.__last_trajectory[idx:])

        # distances = np.linalg.norm(path - path[0], axis=1)
        # target_idx = distances.argmin()
        return idx

    def _create_Task_message(self, path, speed_profile):
        route = Route()
        route.header.stamp = rospy.get_rostime()

        for point, speed in zip(path, speed_profile):
            x, y = point
            pws = PointWithSpeed()
            pws.x = x
            pws.y = y
            pws.speed = speed
            pws.d_time = 0.0
            route.route.append(pws)

        return route

    def _create_message(self, path_to_start, path):
        route = Route()
        route.header.stamp = rospy.get_rostime()

        for x, y in itertools.chain(path_to_start, path[1:]):
            pws = PointWithSpeed()
            pws.x = x
            pws.y = y
            pws.speed = 0.5
            pws.d_time = 0.0
            route.route.append(pws)

        rospy.logwarn(f"{len(route.route)}, {len(path_to_start)}, {len(path)}")

        if len(route.route) > 0:
            path_points = min(len(route.route), 50)
            for i in range(1, path_points + 1):
                route.route[-i].speed = min(i * 0.05, route.route[-i].speed)

        return route


class CovaragePathGeneratorNode(Node):
    def __init__(self, name, frame, *args, **kwargs):
        super(CovaragePathGeneratorNode, self).__init__(name=name,
                                                        run_cb=self.run,
                                                        *args, **kwargs)
        self._frame = frame
        self.__coverage_path_client = CoveragePathClient()
        self.__astar_planner = ContinuosAstarWrapper(frame)

    def run(self, nodedata):
        # rospy.logerr("[CovaragePathGeneratorNode] Enter")

        # source = self.__position2point32()

        # task = self._frame.route_task_task_polygon_message
        # path = self.__coverage_path_client.get_path(
        #     task.target_polygon, source
        # ).path

        path = self._frame.get_coverage_path()
        expanded_path = self._expand_path(path)
        cropped_path = self._crop_path(expanded_path)
        smooth_path = self._smooth_path(cropped_path)
        target_path = self._create_message(smooth_path)
        self._frame.set_trajectory(target_path)

        path_to_start = self.__calc_astar_path_to_target((expanded_path[0].x, expanded_path[1].y))

        rospy.logerr(f"[CovaragePathGeneratorNode] expanded_path {len(expanded_path)}")

        route = Route()
        route.header.stamp = self._frame.get_localization().header.stamp

        for x, y in path_to_start:
            pws = PointWithSpeed()
            pws.x = x
            pws.y = y
            pws.speed = 0.0
            pws.d_time = 0.0
            route.route.append(pws)

        for point in expanded_path:
            pws = PointWithSpeed()
            pws.x = point.x
            pws.y = point.y
            pws.speed = 0.0
            pws.d_time = 0.0
            route.route.append(pws)
        self._frame.set_trajectory(route)

        return NodeStatus(NodeStatus.SUCCESS)

    def __calc_astar_path_to_target(self, target):
        loca = self._frame.get_localization()
        robot_pose = self.__localization2tuple(loca)

        path = self.__astar_planner.search(robot_pose, target)
        return path

    def __localization2tuple(self, pos):
        pos = (pos.pose.x, pos.pose.y, pos.yaw)
        return pos

    def expand_path(self, path):
        expanded_path = list()

        for i in range(1, len(path)):
            p1 = path[i - 1]
            p2 = path[i]

            dist = self.calc_distance_btwn_points(p1, p2)
            if dist > 1.0:
                parts = int(dist / 0.2)
                new_points = self.get_equidistant_points(
                    (p1.x, p1.y),
                    (p2.x, p2.y),
                    parts
                )
                converted_points = self.convert_list_to_point3_array(
                    new_points)
                expanded_path += converted_points
            else:
                expanded_path += [p1, p2]

        return expanded_path

    def get_equidistant_points(self, p1, p2, parts):
        return list(zip(np.linspace(p1[0], p2[0], parts+1),
                        np.linspace(p1[1], p2[1], parts+1)))

    def calc_distance_btwn_points(self, p1, p2):
        v1 = np.array([p1.x, p1.y])
        v2 = np.array([p2.x, p2.y])
        return np.linalg.norm(v1 - v2)

    def convert_list_to_point3_array(self, path):
        new_path = list()

        for point in path:
            p = Point32(
                x=point[0],
                y=point[1]
            )
            new_path.append(p)
        return new_path

    def __position2point32(self):
        pos = self._frame.get_localization()
        pos32 = Point32()
        pos32.x = pos.pose.x
        pos32.y = pos.pose.y
        return pos32
