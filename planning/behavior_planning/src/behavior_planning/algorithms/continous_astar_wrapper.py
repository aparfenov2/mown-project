
#!/usr/bin/env python3
from time import sleep
from numpy.lib.function_base import append
import rospy
import numpy as np

from continious_state_planner import AstarPathPlanner, CurveGenerator


class ContinuosAstarWrapper(object):
    def __init__(self, frame):
        self.frame = frame

    def search(self, source_point, goal_point):
        scale = self.frame.get_grid_map_scale()
        if scale is None:
            rospy.logwarn("[AstarWrapper] We can not calculate path because we don't have a map scale.")
            return []

        obstacles = self.frame.get_grid_map_obstacles()
        grid_source = self.convert_to_grid(source_point)
        robo_yaw = self.frame.get_robot_yaw()
        grid_source = (
            grid_source[0],
            grid_source[1],
            robo_yaw
        )
        grid_goal = self.convert_to_grid(goal_point)

        curve_generator = CurveGenerator()
        astar = AstarPathPlanner(curve_generator)
        path = astar.astar_statespace(grid_source, grid_goal)
        
        result = []
        for point in path:
            result.append(self.convert_to_coordinate(point))

        return result

    def convert_to_grid(self, point):
        scale = self.frame.get_grid_map_scale()
        start_pos = self.frame.get_grid_map_start_pos()

        return tuple(
            [int((point[i] - start_pos[i]) / scale) for i in range(2)]
        )

    def convert_to_coordinate(self, point):
        scale = self.frame.get_grid_map_scale()
        start_pos = self.frame.get_grid_map_start_pos()

        return [
            float((point[i] + 0.5) * scale) + start_pos[i] for i in range(2)
        ]
