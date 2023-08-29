from threading import RLock

import numpy as np

from .discrete_trajectory import DiscreteTrajectory

from engix_msgs.msg import ControlDebug


class Frame(object):
    def __init__(self):
        self._rlock = RLock()
        self._localization = None
        self._route = None
        self._route_as_list = []
        self._discrete_trajectory = DiscreteTrajectory()

        self._debug = ControlDebug()
        self._path = Path()
        self._state = State()

        self._sonar_range = 10000000.0

    def reset_debug(self):
        self._debug = ControlDebug()

    @property
    def state(self):
        return self._state

    @property
    def path(self):
        return self._path

    @property
    def control_debug(self):
        return self._debug

    @property
    def discrete_trajectory(self):
        return self._discrete_trajectory

    def lock(self):
        return self._rlock

    def receive_route(self, message):
        with self._rlock:
            self._route = message
            self.__trajectory_to_list()
            self._discrete_trajectory.process_route(message)
            self._path.update(message)

    def receive_localization(self, message):
        with self._rlock:
            self._localization = message
            self._state = State(
                self._localization.pose.x,
                self._localization.pose.y,
                self._localization.yaw,
                self._localization.speed,
                self._localization.angular_speed
            )

    def __trajectory_to_list(self):
        path_as_list = list()

        for point in self._route.route:
            path_as_list.append([point.x, point.y, point.speed])

        self._route_as_list = path_as_list

    def get_trajectory_as_list(self):
        return self._route_as_list

    def has_localization(self):
        return self._localization is not None

    def has_trajectory(self):
        return self._route is not None and len(self._route.route) != 0

    def get_robot_pose(self):
        return (self._localization.pose.x,
                self._localization.pose.y,
                self._localization.yaw)

    def get_robot_yaw(self):
        return self._localization.yaw

    def get_robot_location(self):
        return (self._localization.pose.x,
                self._localization.pose.y)

    def get_robot_speed(self):
        return self._localization.speed

    def get_robot_angular_speed(self):
        return self._localization.angular_speed

    def receive_range_sensor(self, message):
        self._sonar_range = message.range

    @property
    def range_data(self):
        return self._sonar_range


class State:
    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0, w=0.0):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v
        self.w = w


class Path:
    def __init__(self) -> None:
        self._x = []
        self._y = []
        self._yaw = []
        self._v = []
        self._k = []

        self._has_path = False

        self._epsilon = 10e-6

    @property
    def path_len(self):
        return len(self._x)

    @property
    def has_path(self):
        return self._has_path

    @property
    def x(self):
        return self._x

    @property
    def y(self):
        return self._y

    @property
    def yaw(self):
        return self._yaw

    @property
    def speed(self):
        return self._v

    @property
    def kappa(self):
        return self._k

    def update(self, message):
        if len(message.route) == 0:
            self._has_path = False
            return
        self._x = [0.0] * len(message.route)
        self._y = [0.0] * len(message.route)
        self._yaw = [0.0] * len(message.route)
        self._v = [0.0] * len(message.route)
        self._k = [0.0] * len(message.route)

        for index, point in enumerate(message.route):
            self._x[index] = point.x
            self._y[index] = point.y
            self._v[index] = point.speed

        self._update_kappas()

        self._has_path = True

    def _update_kappas(self):
        distance = 0.0
        arc_lengths = [0.0] * len(self._x)
        arc_lengths[0] = distance

        cur_x = self._x[0]
        cur_y = self._y[0]
        next_x = 0
        next_y = 0
        for i, (x, y) in enumerate(zip(self._x, self._y)):
            next_x = x
            next_y = y
            distance += np.sqrt((cur_x - next_x)**2 + (cur_y - next_y)**2)
            arc_lengths[i] = distance
            cur_x = next_x
            cur_y = next_y

        x_primes = [0.0] * len(self._x)
        y_primes = [0.0] * len(self._y)
        for i in range(len(self._x)):
            dx = 0
            dy = 0
            ds = 0
            if i == 0:
                dx = self._x[i + 1] - self._x[i]
                dy = self._y[i + 1] - self._y[i]
                ds = arc_lengths[i + 1] - arc_lengths[i]
            elif i == len(self._x) - 1:
                dx = self._x[i] - self._x[i - 1]
                dy = self._y[i] - self._y[i - 1]
                ds = arc_lengths[i] - arc_lengths[i - 1]
            else:
                # dx = self._x[i + 1] - self._x[i - 1]
                # dy = self._y[i + 1] - self._y[i - 1]
                # ds = arc_lengths[i + 1] - arc_lengths[i - 1]
                dx = self._x[i + 1] - self._x[i]
                dy = self._y[i + 1] - self._y[i]
                ds = arc_lengths[i + 1] - arc_lengths[i]

            x_primes[i] = dx / ds
            y_primes[i] = dy / ds
            self._yaw[i] = np.arctan2(dy, dx)

        for i in range(len(self._x)):
            ddx = 0
            ddy = 0
            ds = 0
            if i == 0:
                ddx = x_primes[i + 1] - x_primes[i]
                ddy = y_primes[i + 1] - y_primes[i]
                ds = arc_lengths[i + 1] - arc_lengths[i]
            elif i == len(self._x) - 1:
                ddx = x_primes[i] - x_primes[i - 1]
                ddy = y_primes[i] - y_primes[i - 1]
                ds = arc_lengths[i] - arc_lengths[i - 1]
            else:
                ddx = x_primes[i + 1] - x_primes[i - 1]
                ddy = y_primes[i + 1] - y_primes[i - 1]
                ds = arc_lengths[i + 1] - arc_lengths[i - 1]

            dxds = x_primes[i]
            dyds = y_primes[i]
            ddxds = ddx / ds
            ddyds = ddy / ds
            self._k[i] = (dxds * ddyds - dyds * ddxds) / (np.power(dxds**2 + dyds**2, 1.5) + self._epsilon)
