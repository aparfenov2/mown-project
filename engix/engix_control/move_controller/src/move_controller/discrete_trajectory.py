from math import sqrt

import numpy as np
from rtree import index
import rospy


class DiscreteTrajectory:
    def __init__(self) -> None:
        self._points = list()
        self._time_table = np.array([])
        self._s_indexes = index.Index()
        self._start_time = None
        self._last_time = None

    def process_route(self, message):
        self._start_time = message.header.stamp.to_sec()
        self._points = [None] * len(message.route)
        self._time_table = np.array([0.0] * len(message.route))
        self._s_indexes = index.Index()

        for i, point in enumerate(message.route):
            self._points[i] = point
            self._time_table[i] = self._points[i].d_time
            self._s_indexes.insert(i, (point.x, point.y))

        self._last_time = self._time_table[-1] if len(self._points) > 0.0 else 0.0

    def calculate_target_points(self, robot_position, dt, t_count):
        # dt_start = t_start - self._start_time
        # dt_start = max(dt_start, 0.0)

        # if dt_start > self._last_time:
        #     return []

        # t = dt_start
        result = []

        dt_start = self._find_nearest_point(robot_position)

        for i in range(t_count):
            t = dt_start + dt * i
            clothest_node_index = self._closest_node(t)
            point = self._calculate_approximated_point_at_t(t, clothest_node_index)
            result.append(point)

        if len(result) < t_count:
            result += [result[-1]] * (t_count - len(result))

        return result

    def _find_nearest_point(self, point):
        nearest_point_index = list(self._s_indexes.nearest((point[0], point[1]), 1))[0]

        return self._points[nearest_point_index].d_time

    def _closest_node(self, t):
        dist_2 = (self._time_table - t)**2
        return np.argmin(dist_2)

    def _calculate_approximated_point_at_t(self, t, nearest_index):
        first_point = None
        second_point = None

        t_first = None
        if nearest_index == 0 or nearest_index == len(self._points) - 1:
            return [
                self._points[nearest_index].x,
                self._points[nearest_index].y,
                self._points[nearest_index].speed
            ]
        elif t > self._time_table[nearest_index]:
            t_first = self._time_table[nearest_index]
            first_point = self._points[nearest_index]
            second_point = self._points[nearest_index + 1]
        else:
            t_first = self._time_table[nearest_index - 1]
            first_point = self._points[nearest_index - 1]
            second_point = self._points[nearest_index]

        ds = t - t_first

        return [
            first_point.x + ds * (second_point.x - first_point.x),
            first_point.y + ds * (second_point.y - first_point.y),
            first_point.speed + ds * (second_point.speed - first_point.speed)
        ]
