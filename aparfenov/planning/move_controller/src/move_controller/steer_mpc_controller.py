
import numpy as np
from scipy import optimize

from .common import convert_orientation, compute_theta, angles_difference


class SteerMPC(object):
    def __init__(self, frame):
        self._frame = frame
        self._horizon = 10
        self._dt = 0.1

        self.bounds = np.array(
            [(-1, 1) for _ in range(self._horizon)]
        )
        self.last_x0 = 0.0

    def __model(self, prev_pos, v, w):
        new_pos = [0.0, 0.0, 0.0]
        new_pos[0] = prev_pos[0] + v * np.cos(prev_pos[2]) * self._dt
        new_pos[1] = prev_pos[1] + v * np.sin(prev_pos[2]) * self._dt
        new_pos[2] = convert_orientation(prev_pos[2] + w * self._dt)

        return new_pos

    def __find_clothest_point(self, trajectory, pos):
        path = trajectory
        cur_pose = np.array(pos[:2])

        idx = (np.linalg.norm(path - cur_pose, axis=1)).argmin()
        path = np.array(trajectory[idx:])
        return idx

    def __distance_between_poses(self, p1, p2):
        p1 = np.array(p1[:2])
        p2 = np.array(p2[:2])
        return np.linalg.norm(p1 - p2)

    def cost_function(self, x, init_pos, v, trajectory):
        cost = 0

        a_w = 1.0
        a_dw = 1.0
        a_dtheta = 1.0
        a_l_err = 1.0

        last_pos = init_pos
        target_traj_id = self.__find_clothest_point(trajectory, last_pos)

        for i in range(self._horizon):
            w = x[i]
            cost += a_w * w * w

            cur_pos = self.__model(last_pos, v, w)

            if i != 0:
                dw = w - x[i - 1]
                if dw > 0.1:
                    cost += 1000 * a_dw * dw * dw
                else:
                    cost += a_dw * dw * dw

                if target_traj_id == trajectory.shape[0] - 1:
                    ps = trajectory[target_traj_id - 1]
                    pe = trajectory[target_traj_id]
                else:
                    ps = trajectory[target_traj_id]
                    pe = trajectory[target_traj_id + 1]

                theta = compute_theta(ps, pe)
                dtheta = angles_difference(theta, cur_pos[2])

                cost += a_dtheta * dtheta * dtheta

            target_traj_id = max(
                self.__find_clothest_point(trajectory, cur_pos),
                target_traj_id
            )
            
            last_pos = cur_pos

            l_error = self.__distance_between_poses(
                cur_pos,
                trajectory[target_traj_id]
            )
                
            cost += a_l_err * l_error * l_error
        return cost

    def execute(self, target):
        trajectory = np.array([p[:2] for p in self._frame.get_trajectory_as_list()])
        x0 = np.array([
            self.last_x0 for _ in range(self._horizon)
        ])
        init_pos = self._frame.get_robot_pose()
        v = self._frame.get_robot_speed()

        solution = optimize.minimize(self.cost_function, 
                                     x0=x0,  
                                     bounds=self.bounds, 
                                     args=(init_pos, v, trajectory),
                                     method='SLSQP',
                                     tol = 1e-5,
                                     options = {'eps': 0.01, 'disp': False})
        if solution.success:
            self.last_x0 = solution.x[0]
            return self.last_x0

        return None
