from threading import RLock

from .discrete_trajectory import DiscreteTrajectory

from enginx_msgs.msg import ControlDebug


class Frame(object):
    def __init__(self):
        self._rlock = RLock()
        self._localization = None
        self._route = None
        self._route_as_list = []
        self._discrete_trajectory = DiscreteTrajectory()

        self._debug = ControlDebug()

    def reset_debug(self):
        self._debug = ControlDebug()

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

    def receive_localization(self, message):
        with self._rlock:
            self._localization = message

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

