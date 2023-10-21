class Frame:
    def __init__(self) -> None:
        self._localization = LocalizationWrapper()
        self._dubins_planning_task = DubinsPlanningTaskWrapper()
        self._line_moving_task = LineMovingTaskWrapper()
        self._coverage_task = CoverageTaskWrapper()

    @property
    def localization(self):
        return self._localization

    @property
    def line_moving_task(self):
        return self._line_moving_task

    @property
    def dubins_planning_task(self):
        return self._dubins_planning_task

    @property
    def coverage_task(self):
        return self._coverage_task

    def set_localization(self, new_localization):
        self._localization.set_message(new_localization)

    def set_line_moving_task(self, message):
        self._line_moving_task.set_message(message)

    def set_dubins_planning_task(self, message):
        self._dubins_planning_task.set_message(message)

    def set_coverage_task(self, message):
        self._coverage_task.set_message(message)


class AbstractMessage:
    def __init__(self) -> None:
        self.message = None

    def set_message(self, message):
        self.message = message


class LocalizationWrapper(AbstractMessage):
    def __init__(self):
        super(LocalizationWrapper, self).__init__()

    def check_message(self) -> bool:
        return self.message is not None

    def get_xy_position(self) -> tuple:
        return (
            self.message.pose.x,
            self.message.pose.y
        )

    def get_yaw(self) -> float:
        return self.message.yaw

    def get_pose(self) -> float:
        return (
            self.message.pose.x,
            self.message.pose.y,
            self.message.yaw
        )


class LineMovingTaskWrapper(AbstractMessage):
    def __init__(self):
        super(LineMovingTaskWrapper, self).__init__()

    @property
    def distance(self):
        return self.message.distance


class DubinsPlanningTaskWrapper(AbstractMessage):
    def __init__(self):
        super(DubinsPlanningTaskWrapper, self).__init__()

    @property
    def step_size(self):
        return self.message.step_size

    @property
    def turning_radius(self):
        return self.message.turning_radius

    @property
    def pose(self):
        return (
            self.message.target_pose.x,
            self.message.target_pose.y,
            self.message.target_pose.theta
        )


class CoverageTaskWrapper(AbstractMessage):
    def __init__(self):
        super(CoverageTaskWrapper, self).__init__()

    @property
    def target_polygon(self):
        return self.message.target_polygon

    @property
    def approximate(self):
        return self.message.approximate

    @property
    def auto_angle(self):
        return self.message.auto_angle

    @property
    def angle(self):
        return self.message.angle

