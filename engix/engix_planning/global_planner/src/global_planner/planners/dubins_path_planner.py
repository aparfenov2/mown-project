from typing import Optional
import numpy as np
import dubins

from .utils import points_to_path
from global_planner.common.frame import Frame

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path


class DubinsPathPlanner:
    def __init__(self, step_size: float = 0.5) -> None:
        self.step_size = step_size

    def plan(self, frame: Frame) -> Optional[Path]:
        current_pose = frame.localization.get_pose()
        target_pose = frame.dubins_planning_task.pose

        turning_radius = frame.dubins_planning_task.turning_radius
        step_size = frame.dubins_planning_task.step_size

        path = dubins.shortest_path(current_pose, target_pose, turning_radius)
        configurations, _ = path.sample_many(step_size)

        path = [(x, y) for x, y, _ in configurations]

        return points_to_path(path)
