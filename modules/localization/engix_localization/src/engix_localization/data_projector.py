from typing import Tuple
import numpy as np

from tf.transformations import euler_from_quaternion, quaternion_from_euler, quaternion_matrix, quaternion_from_matrix

from .utils import quaternion2list, create_transform

from geometry_msgs.msg import TwistStamped, Quaternion


class DataProjector:
    def __init__(self, imu_to_base_transform, gnss_to_base_transform) -> None:
        self.__imu_to_base_transform = imu_to_base_transform
        self.__gnss_to_base_transform = gnss_to_base_transform

    def gnss_to_base_position(self, x: float, y: float, z: float) -> Tuple[float, float, float]:
        # TODO: Remove old code
        # translate_mat = np.ones(shape=(4, 4))
        # translate_mat[0, 3] = x
        # translate_mat[1, 3] = y
        # translate_mat[2, 3] = z

        translate_mat = np.eye(4)
        translate_mat[:3, 3] = [x, y, z]

        base_link = np.dot(translate_mat, self.__gnss_to_base_transform)
        # TODO: Remove old code
        # return (base_link[0, 3], base_link[1, 3], base_link[2, 3])
        return tuple(base_link[:3, 3])

    def yaw_from_imu(self, orientation: Quaternion) -> Tuple[float, np.ndarray]:
        orientation = quaternion2list(orientation)
        rot_mat = quaternion_matrix(orientation)
        base_rot = np.dot(rot_mat, self.__imu_to_base_transform)

        orientation = quaternion_from_matrix(base_rot)

        _, _, yaw = euler_from_quaternion(orientation)
        return (yaw, orientation)
