import math
import numpy as np


def distance(p1, p2):
    return math.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)


def find_nearest_point(robot_position, path) -> int:
    min_distance = float('inf')
    nearest_index = None

    for i, point in enumerate(path):
        d = distance(robot_position, point)
        if d < min_distance:
            min_distance = d
            nearest_index = i

    return nearest_index


class PIDController:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.prev_error = 0
        self.integral = 0

    def compute(self, error):
        self.integral += error
        derivative = error - self.prev_error
        self.prev_error = error
        return self.kp * error + self.ki * self.integral + self.kd * derivative


class PIDTrackingController:
    def __init__(
        self,
        linear_controller_params: dict,
        angular_controller_params: dict,
        max_linear_velocity: float,
        max_acceleration: float,
        max_angular_velocity: float,
        carrot_step: int,
        last_point_threshold: float = 0.05
    ) -> None:
        self.linear_controller = PIDController(
            linear_controller_params['kp'],
            linear_controller_params['ki'],
            linear_controller_params['kd'],
        )

        self.angular_controller = PIDController(
            angular_controller_params['kp'],
            angular_controller_params['ki'],
            angular_controller_params['kd'],
        )

        self.max_linear_velocity = max_linear_velocity
        self.max_acceleration = max_acceleration
        self.max_angular_velocity = max_angular_velocity
        self.carrot_step = carrot_step

        self.last_point_threshold = last_point_threshold

    def find_carrot(self, nearest_index, path):
        next_index = min(len(path) - 1, nearest_index + self.carrot_step)
        return next_index

    def control(self, frame) -> tuple:
        if not frame.has_localization() or not frame.has_path():
            return 0.0, 0.0

        robot_position = frame.localization.get_xy_position()
        yaw = frame.localization.get_yaw()

        path = frame.path.get_path_as_array()
        speed_profile = frame.path.speed_profile

        nearest_index = find_nearest_point(robot_position, path)
        nearest_point = path[nearest_index]

        carrot_index = self.find_carrot(nearest_index, path)

        carrot_point = path[carrot_index]

        x_error = carrot_point[0] - robot_position[0]
        y_error = carrot_point[1] - robot_position[1]

        speed_error = speed_profile[carrot_index] - frame.localization.speed

        distance_error = distance(carrot_point, robot_position)
        angle_error = math.atan2(y_error, x_error) - yaw
        angle_error = math.atan2(math.sin(angle_error), math.cos(angle_error))

        target_acceleration = self.linear_controller.compute(speed_error)
        target_angular_velocity = self.angular_controller.compute(angle_error)

        target_acceleration = min(
            max(target_acceleration, -self.max_acceleration),
            self.max_acceleration
        )
        target_angular_velocity = min(max(
            target_angular_velocity, -self.max_angular_velocity),
            self.max_angular_velocity
        )

        target_linear_speed = frame.localization.speed + target_acceleration
        target_linear_speed = min(
            max(target_linear_speed, 0.0),
            self.max_linear_velocity
        )

        if nearest_index == len(path) - 1 and abs(distance_error) < self.last_point_threshold:
            target_linear_speed = 0.0
            target_angular_velocity = 0.0

        # print(f'speed: {target_linear_speed}, ang_vel: {target_angular_velocity}, target_speed: {speed_profile[carrot_index]}, distance: {distance_error}, carrot_index: {carrot_index}, path_len: {len(path)}')

        return target_linear_speed, target_angular_velocity
