
#!/usr/bin/env python3
from time import sleep
from numpy.lib.function_base import append
import rospy
import numpy as np

from continious_state_planner import AStar


class AstarWrapper(object):
    def __init__(self, frame):
        self.frame = frame

    def search(self, source_point, goal_point):
        scale = self.frame.get_grid_map_scale()
        if scale is None:
            rospy.logwarn("[AstarWrapper] We can not calculate path because we don't have a map scale.")
            return []

        obstacles = self.frame.get_grid_map_obstacles()
        grid_source = self.convert_to_grid(source_point)
        grid_goal = self.convert_to_grid(goal_point)

        astar = AStar(grid_source, grid_goal, "euclidean", obstacles=obstacles)
        path, visited = astar.searching()
        
        result = []
        for point in path:
            result.append(self.convert_to_coordinate(point))

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
