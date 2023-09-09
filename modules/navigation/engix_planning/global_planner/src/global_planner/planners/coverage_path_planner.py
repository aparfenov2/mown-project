from global_planner.common.frame import Frame
from .utils import points_to_path, yaw_from_pose

from shapely.geometry import Polygon
import numpy as np
import dubins

from coverage_path_planner.area_polygon import AreaPolygon

from nav_msgs.msg import Path


class DubinsPlannerHelper:
    def __init__(self, step_size: float = 0.1, turning_radius: float = 0.5) -> None:
        self.__step_size = step_size
        self.__turning_radius = turning_radius

    def plan(self, robot_pose, target_pose) -> Path:
        path = dubins.shortest_path(robot_pose, target_pose, self.__turning_radius)
        configurations, _ = path.sample_many(self.__step_size)

        path = [(x, y) for x, y, _ in configurations]

        return points_to_path(path)


class CoveragePathPlanner:
    STATE_PLAN_TO_START = "plan_to_start"
    STATE_COVERAGE_PLAN = "coverage_plan"

    def __init__(self) -> None:
        self.__state = self.STATE_PLAN_TO_START
        self.__path_to_start = None
        self.__coverage_path = None
        self.__polygon = None
        self.__coverage_path_listed = None
        self.__coverage_plan_send_check = False
        self.__dubins_helper = DubinsPlannerHelper()
        self.__reach_path_end_threshold = 0.3
        self.__step_size = 0.5
        self.__offset_distance = -0.2
        self.__offset_resolution = 3.0

        self.__range_left = 2
        self.__range_right = 25
        self.__prev_nearest_index = 0

    @property
    def polygon(self):
        return self.__polygon

    @property
    def path_to_start(self):
        return self.__path_to_start

    @property
    def coverage_path(self):
        return self.__coverage_path

    def plan(self, frame: Frame) -> Path:
        if self.__state == self.STATE_PLAN_TO_START:
            return self.__plan_to_start(frame)
        elif self.__state == self.STATE_COVERAGE_PLAN:
            return self.__coverage_plan(frame)
        else:
            raise RuntimeError(f"Got wrong CoveragePathPlanner state: {self._state}")

    def __calculate_target_yaw(self, poses):
        cur = poses[0].pose.position
        next = poses[1].pose.position

        yaw = np.arctan2(next.y - cur.y, next.x - cur.x)
        return yaw

    def __plan_to_start(self, frame: Frame) -> None:
        if self.__path_to_start is None:
            self.__coverage_path = self.__create_coverage_path(frame)
            position = self.__coverage_path.poses[0].pose.position
            yaw = self.__calculate_target_yaw(self.__coverage_path.poses)
            target_pose = (position.x, position.y, yaw)
            self.__path_to_start = self.__create_dubins_to_pose(frame, target_pose)

            return self.__path_to_start

        robot_position = frame.localization.get_xy_position()
        if self.__check_end_reach(robot_position, self.__path_to_start):
            self.__state = self.STATE_COVERAGE_PLAN

        return None

    def __check_end_reach(self, robot_position: set, path: Path) -> bool:
        position = path.poses[-1].pose.position
        last_point = (position.x, position.y)
        if (
            np.linalg.norm(np.array(robot_position) - np.array(last_point))
            < self.__reach_path_end_threshold
        ):
            return True

        return False

    def __coverage_plan(self, frame: Frame) -> None:
        if not self.__coverage_plan_send_check:
            self.__coverage_plan_send_check = True
            self.__prev_nearest_index = 0
            sub_path = points_to_path(self.__coverage_path_listed[: self.__range_right])
            return sub_path

        left_index = max(0, self.__prev_nearest_index - self.__range_left)
        right_index = min(
            len(self.__coverage_path_listed) - 1,
            self.__prev_nearest_index + self.__range_right,
        )
        robot_position = frame.localization.get_xy_position()
        nearest_index = self.__find_nearest_index_at_path(
            robot_position, self.__coverage_path_listed[left_index : right_index + 1]
        )
        index = max(nearest_index + left_index, self.__prev_nearest_index)
        self.__prev_nearest_index = index

        left_index = max(0, index - self.__range_left)
        right_index = min(len(self.__coverage_path_listed) - 1, index + self.__range_right)
        sub_path = points_to_path(
            self.__coverage_path_listed[left_index : right_index + 1]
        )
        return sub_path

    def __find_nearest_index_at_path(self, position, path):
        cur_pos = np.array(path)
        position = np.array(position)

        idx = (np.linalg.norm(position - cur_pos, axis=1)).argmin()

        return idx

    def __create_dubins_to_pose(self, frame: Frame, target_pose):
        robot_pose = frame.localization.get_pose()
        return self.__dubins_helper.plan(robot_pose, target_pose)

    def __create_coverage_path(self, frame: Frame) -> None:
        robot_position = frame.localization.get_xy_position()
        message = frame.coverage_task

        listed_polygon = [(point.x, point.y) for point in message.target_polygon.points]
        listed_polygon = self.__create_offset(listed_polygon)

        if message.approximate:
            temp_polygon = listed_polygon
            listed_polygon = list()

            for i in range(1, len(temp_polygon)):
                listed_polygon.append(temp_polygon[i - 1])
                point_count = np.linalg.norm(
                    np.array(temp_polygon[i]) - np.array(temp_polygon[i - 1])
                )
                point_count = max(1, int(point_count))
                extended_segment = np.linspace(
                    temp_polygon[i - 1], temp_polygon[i], point_count
                ).tolist()
                listed_polygon += extended_segment

        if message.auto_angle:
            path_creator = AreaPolygon(
                listed_polygon, robot_position, ft=self.__step_size
            )
        else:
            path_creator = AreaPolygon(
                listed_polygon,
                robot_position,
                ft=self.__step_size,
                angle=message.angle,
            )

        coverage_route_points = path_creator.get_area_coverage()
        path = [(p[0], p[1]) for p in coverage_route_points.coords]
        path = self.__approximate_list(path)

        self.__polygon = listed_polygon
        # self.__coverage_path_listed = self.__approximate_list(path)
        self.__coverage_path_listed = path

        return points_to_path(path)
    
    def __approximate_list(self, path_list: list) -> None:
        temp_list = path_list
        appr_list = list()
        appr_list.append(temp_list[0])

        for i in range(1, len(temp_list)):
            distance_between = np.linalg.norm(
                np.array(temp_list[i]) - np.array(temp_list[i - 1])
            )
            point_count = max(1, int(10.0 * distance_between))
            extended_segment = np.linspace(
                temp_list[i - 1], temp_list[i], point_count + 1
            ).tolist()
            appr_list += extended_segment[1:]

        return appr_list

    def __create_offset(self, polygon):
        shapely_polygon = Polygon(polygon)
        poly_line_offset = shapely_polygon.buffer(
            self.__offset_distance,
            resolution=self.__offset_resolution,
            join_style=2,
            mitre_limit=1,
        ).exterior
        return [(point[0], point[1]) for point in poly_line_offset.coords]

    def reset(self) -> None:
        self.__state = self.STATE_PLAN_TO_START
        self.__path_to_start = None
        self.__coverage_path = None
        self.__polygon = None
        self.__coverage_plan_send_check = False
