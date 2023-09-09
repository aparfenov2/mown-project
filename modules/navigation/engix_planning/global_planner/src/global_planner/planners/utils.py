from typing import Optional
import numpy as np
import tf

from geometry_msgs.msg import PoseStamped, Pose
from nav_msgs.msg import Path


def points_to_path(points: list) -> Path:
    path = Path()
    for point in points:
        pose_stamped = PoseStamped()
        pose_stamped.pose.position.x = point[0]
        pose_stamped.pose.position.y = point[1]

        path.poses.append(pose_stamped)

    return path


def yaw_from_pose(pose: Pose) -> float:
    quaternion = (
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z,
        pose.orientation.w,
    )
    roll, pitch, yaw = tf.transformations.euler_from_quaternion(quaternion)
    return yaw
