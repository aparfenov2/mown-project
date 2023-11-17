#! /usr/bin/env python
import math
from typing import Tuple, List

import numpy as np
import rospy
import tf
from tf.transformations import translation_matrix, quaternion_matrix

from abstractnode import AbstractNode

from .wgs_convertor import convert_to_utm

from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from engix_msgs.msg import Localization


import tf2_ros
from geometry_msgs.msg import TransformStamped


def quaternion2list(quaternion) -> Tuple[float, float, float, float]:
    return (quaternion.x, quaternion.y, quaternion.z, quaternion.w)


def create_transform(trans, rot) -> np.ndarray:
    Rq = quaternion_matrix(rot)
    T = translation_matrix(trans)
    return Rq + T
