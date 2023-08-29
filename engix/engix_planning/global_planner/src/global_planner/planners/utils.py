from typing import Optional
import numpy as np

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path


def points_to_path(points: list) -> Path:
    path = Path()
    for point in points:
        pose_stamped = PoseStamped()
        pose_stamped.pose.position.x = point[0]
        pose_stamped.pose.position.y = point[1]

        path.poses.append(pose_stamped)

    return path
