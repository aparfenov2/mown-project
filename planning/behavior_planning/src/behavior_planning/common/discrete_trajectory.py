from math import sqrt

import numpy as np
from rtree import index
import rospy


class DiscreteTrajectory:
    def __init__(self) -> None:
        self._points = list()
        self._s_indexes = index.Index()
        self._s_array = None
        self._s_table = list()
        self._max_s = 0.0

    def set_points(self, points):
        self._points = points
        self._s_table = [0] * len(self._points)
        last_point = None
        s = 0.0
        for i_index, point in enumerate(self._points[1:]):
            if last_point is None:
                last_point = point
            else:
                s += self.calculate_distance(last_point, point)
                last_point = point
            self._s_table[i_index] = s
            # self._s_indexes.insert(i_index, (s,))
        self._s_array = np.array(self._s_table)
        self._max_s = s
        # rospy.logwarn(f"max s - {self._max_s}, table_len - {len(self._s_table)}")

    def get_max_s(self):
        return self._max_s

    def get_point_at_s(self, s):
        if s <= 0.0:
            return self._points[0]

        if s >= self._max_s:
            return self._points[-1]

        nearest_index = self._closest_node(s)
        new_point = self._calculate_approximated_point_at_s(s, nearest_index)

        return new_point

    def _closest_node(self, s):
        dist_2 = (self._s_array - s)**2
        return np.argmin(dist_2)

    @staticmethod
    def calculate_distance(point1, point2):
        return np.linalg.norm(np.array(point1) - np.array(point2))

    def _calculate_approximated_point_at_s(self, s, nearest_index):
        first_point = None
        second_point = None

        s_first = None

        if s > self._s_table[nearest_index]:
            s_first = self._s_table[nearest_index]
            first_point = self._points[nearest_index]
            second_point = self._points[nearest_index + 1]
        else:
            s_first = self._s_table[nearest_index - 1]
            first_point = self._points[nearest_index - 1]
            second_point = self._points[nearest_index]

        ds = s - s_first

        return [
            first_point[0] + ds * (second_point[0] - first_point[0]),
            first_point[1] + ds * (second_point[1] - first_point[1])
        ]
