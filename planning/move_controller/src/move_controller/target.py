import numpy as np


class Target(object):
    def __init__(self, frame, target_length) -> None:
        self._frame = frame
        self._target_length = target_length

        self._target_pos = None
        self._target_index = None

        self._robot_pos_at_path = None
        self._robot_pos_at_path_index = None

        self._robot_to_target_distance = 0

    def prepare_data(self):
        self.__calculate_target()
        self.__calculate_robot_to_target_distance()

    def __calculate_target(self):
        trajectory = self._frame.get_trajectory_as_list()
        current_pos = self._frame.get_robot_pose()

        if len(trajectory) == 0:
            self._target_pos = None
            self._target_index = 0
            return


        path = np.array([p[:2] for p in trajectory])
        cur_pose = np.array(current_pos[:2])

        idx = (np.linalg.norm(path - cur_pose, axis=1)).argmin()
        path = np.array(trajectory[idx:])

        self._robot_pos_at_path = trajectory[idx:]
        self._robot_pos_at_path_index = idx

        if len(path) == 1:
            self._target_pos = path[0]
            self._target_index = 0
            return

        distances = np.linalg.norm(path - path[0], axis=1)
        distances = np.abs(distances - self._target_length)
        target_idx = distances.argmin()
        self._target_pos = path[target_idx]
        self._target_index = target_idx

    def __calculate_robot_to_target_distance(self):
        trajectory = self._frame.get_trajectory_as_list()
        trajectory = np.array(trajectory)
        sums = 0
        if self._robot_pos_at_path_index == self._target_index:
            self._robot_to_target_distance = 0
            return
    
        for i in range(self._robot_pos_at_path_index + 1, self._target_index + 1):
            p1 = trajectory[i - 1][:2]
            p2 = trajectory[i][:2]
            sums += np.linalg.norm(p1 - p2)

        self._robot_to_target_distance = sums

    def get_robot_to_target_distance(self):
        return self._robot_to_target_distance

    def get_target_index(self):
        return self._target_index

    def get_target_point(self):
        return self._target_pos[:2]

    def get_current_index(self):
        return self._robot_pos_at_path_index

    def get_target_speed(self):
        return self._target_pos[2]

    