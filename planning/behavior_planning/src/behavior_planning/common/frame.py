from threading import RLock
from rtree import index

from task_behavior_engine.tree import Blackboard

from .discrete_trajectory import DiscreteTrajectory
from behavior_planning.algorithms.bresenham_circle import BresenhamCircle

from enginx_msgs.msg import PlanningDebug, SpeedGeneratorDebug


class PlannerBlackboard(Blackboard):
    def __init__(self, frame, *args, **kwargs) -> None:
        super().__init__(*args, **kwargs)
        self._frame = frame

    @property
    def frame(self):
        return self._frame


class DebugData:
    def __init__(self) -> None:
        self.coverage_path = None


class Frame(object):
    TASK_TYPE_INIT = "init_type"
    TASK_TYPE_PLAN_TO_GOAL = 'plan_to_goal_type'
    TASK_TYPE_COV_PLAN = 'cov_plan_type'

    def __init__(self):
        self._task_plan_poly = None
        self._task_plan_to_goal = None
        self._current_task = None
        self._current_task_type = self.TASK_TYPE_INIT

        self._grid_map_scale = 0.1
        self._grid_map_start_point = (0.0, 0.0)
        self._grid_map_obstacles = set()

        self._robot_radius = 0.1

        self._rlock = RLock()

        self._localization_msg = None

        self._trajectory = None

        self._discrete_trajectory = DiscreteTrajectory()
        self._astar_path = None
        self._astar_path_rtree = None

        self._planning_debug = PlanningDebug()
        self._debug_data = DebugData()

        self._path_done = False
        self._cov_path_done = False
        self._cov_task = False

        self._planning_task_type = PlanningTaskTypeWrapper()
        self._localization = LocalizationWrapper()
        self._line_moving_task = LineMovingTaskWrapper()
        self._circle_moving_task = CircleMovingTaskWrapper()
        self._coverage_planning_task = CoveragePlanningTask()

    @property
    def coverage_planning_task(self):
        return self._coverage_planning_task

    @property
    def circle_moving_task(self):
        return self._circle_moving_task

    @property
    def line_moving_task(self):
        return self._line_moving_task

    @property
    def localization(self):
        return self._localization

    @property
    def planning_task_type(self):
        return self._planning_task_type

    def resete_debug(self):
        self._debug_data.coverage_path = None

    @property
    def debug_data(self):
        return self._debug_data

    def get_cov_task(self):
        return self._cov_task

    def set_cov_task(self, new_flag):
        self._cov_task = new_flag

    def get_cov_path_done(self):
        return self._cov_path_done

    def set_cov_path_done(self, new_flag):
        self._cov_path_done = new_flag

    def get_path_done(self):
        return self._path_done

    def set_path_done(self, new_flag):
        self._path_done = new_flag

    def has_astar_path(self):
        return self._astar_path is None

    def set_astar_path(self, new_path):
        self._astar_path = new_path
        self._astar_path_rtree = index.Index()

        for i, (x, y) in enumerate(new_path):
            self._astar_path_rtree.insert(i, (x, y))

    @property
    def astar_path(self):
        return self._astar_path

    @property
    def astar_path_rtree(self):
        if self._astar_path is None:
            return None
        return self._astar_path_rtree

    @property
    def planning_debug(self):
        return self._planning_debug

    @property
    def discrete_trajectory(self):
        return self._discrete_trajectory

    def set_trajectory(self, new_trajectory):
        self._trajectory = new_trajectory

    @property
    def trajectory(self):
        return self._trajectory

    def has_trajectory(self):
        return self._trajectory is not None

    def reset_trajectory(self):
        self._trajectory = None
        self.resete_debug()

    def lock(self):
        return self._rlock

    @property
    def route_task_to_point_message(self):
        return self._task_plan_to_goal

    def set_route_task_to_point_message(self, message):
        self._task_plan_to_goal = message
        self.set_path_done(False)

    def has_route_task_to_point_message(self):
        return self._task_plan_to_goal is not None

    def reset_route_task_to_point_message(self):
        self._task_plan_to_goal = None

    @property
    def route_task_task_polygon_message(self):
        return self._task_plan_poly

    def set_route_task_polygon_message(self, message):
        self._task_plan_poly = message
        self.set_cov_path_done(False)
        self.set_cov_task(True)

    def reset_route_task_polygon_message(self):
        self._task_plan_poly = None

    def has_route_task_polygon_message(self):
        return self._task_plan_poly is not None

    def set_localization(self, message):
        self._localization_msg = message
        self._localization.receive_message(message)

    def has_localization(self):
        return self._localization_msg is not None

    def receive_map_message(self, message):
        with self._rlock:
            width = message.info.width
            height = message.info.height
            self._grid_map_scale = message.info.resolution

            start_pos = message.info.origin.position
            self._grid_map_start_point = (start_pos.x, start_pos.y)

            # for x in range(width):
            #     for y in range(height):
            #         if message.data[x + width * y] == 1:
            #             pass

            obstacles = set()

            for y in range(height):
                for x in range(width):
                    if message.data[y + width * x] > 0:
                        obstacles.add((y, x))

                        not_safe_zone = BresenhamCircle(
                            int(self._robot_radius / self._grid_map_scale),
                            y,
                            x
                        )
                        obstacles = obstacles | not_safe_zone

            self._grid_map_obstacles = obstacles

    def get_localization(self):
        return self._localization_msg

    @property
    def linear_speed(self):
        return self._localization_msg.speed

    @property
    def linear_acceleration(self):
        return self._localization_msg.linear_acceleration

    def get_robot_yaw(self):
        return self._localization_msg.yaw

    def get_grid_map_obstacles(self):
        return self._grid_map_obstacles

    def get_grid_map_scale(self):
        return self._grid_map_scale

    def get_grid_map_start_pos(self):
        return self._grid_map_start_point

    def get_current_task_type(self):
        return self._current_task_type


class LocalizationWrapper:
    def __init__(self) -> None:
        self._message = None

    def receive_message(self, message):
        self._message = message

    def has_localization(self):
        return self._message is not None

    @property
    def stamp(self):
        return self._message.header.stamp

    @property
    def stamp_sec(self):
        return self._message.header.stamp.to_sec()

    @property
    def pose(self):
        return (self._message.pose.x,
                self._message.pose.y,
                self._message.yaw)

    @property
    def position(self):
        return (self._message.pose.x, self._message.pose.y)

    @property
    def linear_speed(self):
        return self._message.speed

    @property
    def linear_acceleration(self):
        return self._message.linear_acceleration

    @property
    def yaw(self):
        return self._message.yaw


class PlanningTaskTypeWrapper:
    def __init__(self) -> None:
        self._message = None

    def receive_message(self, message):
        self._message = message

    def has_message(self):
        return self._message is not None

    @property
    def type(self):
        return self._message.type

    @property
    def stamp_sec(self):
        return self._message.header.stamp.to_sec()


class LineMovingTaskWrapper:
    def __init__(self) -> None:
        self._message = None

    @property
    def distance(self):
        return self._message.distance

    @property
    def target_speed(self):
        return self._message.target_speed

    def receive_message(self, message):
        self._message = message

    def has_message(self):
        return self._message is not None

    @property
    def stamp_sec(self):
        return self._message.header.stamp.to_sec()


class CircleMovingTaskWrapper:
    def __init__(self) -> None:
        self._message = None

    @property
    def left_radius(self):
        return self._message.left_radius

    @property
    def right_radius(self):
        return self._message.right_radius

    @property
    def target_speed(self):
        return self._message.target_speed

    def receive_message(self, message):
        self._message = message

    def has_message(self):
        return self._message is not None

    @property
    def stamp_sec(self):
        return self._message.header.stamp.to_sec()


class DubinsPlanningTaskWrapper:
    def __init__(self) -> None:
        self._message = None

    @property
    def target_pose(self):
        return (self._message.target_pose.x,
                self._message.target_pose.y,
                self._message.target_pose.theta)

    @property
    def turning_radius(self):
        return self._message.turning_radius

    @property
    def step_size(self):
        return self._message.step_size

    @property
    def target_speed(self):
        return self._message.target_speed

    def receive_message(self, message):
        self._message = message

    def has_message(self):
        return self._message is not None

    @property
    def stamp_sec(self):
        return self._message.header.stamp.to_sec()


class CoveragePlanningTask:
    def __init__(self) -> None:
        self._message = None

    @property
    def path(self):
        return [(pose.pose.position.x, pose.pose.position.y)
                for pose in self._message.path.poses]

    @property
    def target_speed(self):
        return self._message.target_speed

    @property
    def turning_radius(self):
        return self._message.turning_radius

    @property
    def step_size(self):
        return self._message.step_size

    def receive_message(self, message):
        self._message = message

    def has_message(self):
        return self._message is not None

    @property
    def stamp_sec(self):
        return self._message.header.stamp.to_sec()
