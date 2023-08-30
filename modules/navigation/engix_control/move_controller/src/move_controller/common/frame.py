class Frame(object):
    def __init__(self):
        self._localization = LocalizationWrapper()
        self._path = PathWrapper()

    @property
    def localization(self):
        return self._localization

    @property
    def path(self):
        return self._path

    def has_localization(self):
        return self._localization.has_message()

    def has_path(self):
        return self._path.has_message()

    def set_localization(self, message):
        self._localization.set_message(message)

    def set_path(self, message):
        self._path.set_message(message)

    def set_speed_profile(self, speeds):
        self._path.set_speed_profile(speeds)


class LocalizationWrapper:
    def __init__(self) -> None:
        self.message = None

    @property
    def speed(self):
        return self.message.speed

    def get_xy_position(self) -> tuple:
        return (
            self.message.pose.x,
            self.message.pose.y
        )

    @property
    def yaw(self) -> float:
        return self.message.yaw

    def get_yaw(self) -> float:
        return self.message.yaw

    def set_message(self, message):
        self.message = message

    def has_message(self):
        return self.message is not None


class PathWrapper:
    def __init__(self) -> None:
        self.message = None
        self.speeds = None

    @property
    def speed_profile(self):
        return self.speeds

    def get_path_as_array(self):
        return [(pose.pose.position.x, pose.pose.position.y) for pose in self.message.poses]

    def set_message(self, message):
        self.message = message

    def has_message(self):
        return self.message is not None

    def set_speed_profile(self, speeds):
        self.speeds = speeds
