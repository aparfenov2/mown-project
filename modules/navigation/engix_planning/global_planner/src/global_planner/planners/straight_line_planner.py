from typing import Optional
import numpy as np

from .utils import points_to_path

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path


class StraightLinePlanner:
    def __init__(self, step_size: float = 0.5) -> None:
        self.step_size = step_size

    def plan(self, frame) -> Optional[Path]:
        current_position = frame.localization.get_xy_position()
        yaw = frame.localization.get_yaw()
        distance = frame.line_moving_task.distance

        target_point = (
            current_position[0] + distance * np.cos(yaw),
            current_position[1] + distance * np.sin(yaw)
        )

        if distance < self.step_size:
            return self.points_to_path([current_position, target_point])

        points_between = self.generate_points_between(current_position,
                                                      target_point,
                                                      self.step_size)

        return points_to_path(points_between)

    def generate_points_between(self, p1, p2, step_size):
        """
        Generate a list of points between two 2D points with a given step size.

        Parameters:
            p1 (tuple): First 2D point (x, y).
            p2 (tuple): Second 2D point (x, y).
            step_size (float): Step size between points.

        Returns:
            list: List of 2D points between p1 and p2 with the given step size.
        """
        x1, y1 = p1
        x2, y2 = p2
        num_points = int(abs(x2 - x1) / step_size) + 1

        if x1 == x2:
            # Handle vertical line
            return [(x1, y1 + i * step_size) for i in range(num_points)]

        slope = (y2 - y1) / (x2 - x1)
        intercept = y1 - slope * x1

        points = [(x1 + i * step_size, slope * (x1 + i * step_size) + intercept)
                  for i in range(num_points)]
        return points


if __name__ == "__main__":
    pass
