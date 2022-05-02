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

        self._localization = None

        self._trajectory = None

        self._discrete_trajectory = DiscreteTrajectory()
        self._astar_path = None
        self._astar_path_rtree = None

        self._planning_debug = PlanningDebug()

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

    def lock(self):
        return self._rlock

    @property
    def route_task_to_point_message(self):
        return self._task_plan_to_goal

    def set_route_task_to_point_message(self, message):
        self._task_plan_to_goal = message

    def has_route_task_to_point_message(self):
        return self._task_plan_to_goal is not None

    def reset_route_task_to_point_message(self):
        self._task_plan_to_goal = None

    @property
    def route_task_task_polygon_message(self):
        return self._task_plan_poly

    def set_route_task_polygon_message(self, message):
        self._task_plan_poly = message

    def reset_route_task_polygon_message(self):
        self._task_plan_poly = None

    def has_route_task_polygon_message(self):
        return self._task_plan_poly is not None

    def set_localization(self, message):
        self._localization = message

    def has_localization(self):
        return self._localization is not None

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
        return self._localization

    @property
    def linear_speed(self):
        return self._localization.speed

    @property
    def linear_acceleration(self):
        return self._localization.linear_acceleration

    def get_robot_yaw(self):
        return self._localization.yaw

    def get_grid_map_obstacles(self):
        return self._grid_map_obstacles

    def get_grid_map_scale(self):
        return self._grid_map_scale

    def get_grid_map_start_pos(self):
        return self._grid_map_start_point

    def get_current_task_type(self):
        return self._current_task_type
