import math

import rospy
import numpy as np
import scipy.linalg as la

from .common import pi_2_pi

from std_msgs.msg import Float32
from geometry_msgs.msg import Twist


class LeonardoController(object):
    def __init__(self, frame, params):
        self._frame = frame

        self._speed_pid = PIDController(params['speed_pid'])
        self._position_pid = PIDController(params['position_pid'])

        self._dt = params['dt']
        self._carrot = params['carrot']
        self._range_sensor_threshold = params['range_sensor_threshold']

        self._steer_lqr = SteerLQR(1.0, 1.0, self._dt)

    def execute(self):
        trajectory = self._frame.path
        nearest_index, _ = self._calc_nearest_index()

        target_index = min(nearest_index + self._carrot, trajectory.path_len - 1)
        current_v = trajectory.speed[target_index]
        robot_speed = self._frame.get_robot_speed()

        if self._frame.range_data < self._range_sensor_threshold:
            current_v = 0.0

        speed_error = (current_v - robot_speed)
        # position_error = (trajectory.speed[target_index] - self._frame.get_robot_speed())

        a = self._speed_pid.execute(speed_error)

        w = self._steer_lqr.execute(self._frame.state, self._frame.path)

        out_speed = robot_speed + a * self._dt
        out_speed = 0.0 if out_speed < 0.0 else out_speed

        print(f'[LeonardoController] {robot_speed=} {out_speed=}, {current_v=}, {a=}')

        return out_speed, w

    def _calc_nearest_index(self):
        state = self._frame.state
        path = self._frame.path

        dx = [state.x - icx for icx in path.x]
        dy = [state.y - icy for icy in path.y]

        d = [idx ** 2 + idy ** 2 for (idx, idy) in zip(dx, dy)]

        mind = min(d)

        ind = d.index(mind)

        mind = math.sqrt(mind)

        dxl = path.x[ind] - state.x
        dyl = path.y[ind] - state.y

        angle = pi_2_pi(path.yaw[ind] - math.atan2(dyl, dxl))
        if angle < 0:
            mind *= -1

        return ind, mind


class PIDController(object):
    def __init__(self, controller_params):
        self.p = controller_params.get('p', 0.0)
        self.i = controller_params.get('i', 0.0)
        self.d = controller_params.get('d', 0.0)

        self.last_error = 0
        self.integrate_error = 0

        self.min_value = controller_params['min_value']
        self.max_value = controller_params['max_value']

        self.min_integral_error = controller_params['min_integral_error']
        self.max_integral_error = controller_params['max_integral_error']

    def execute(self, error):
        d_error = error - self.last_error
        self.integrate_error += error
        self.last_error = error

        self.integrate_error = self.__bound(self.integrate_error, self.min_integral_error, self.max_integral_error)

        value = self.p * error + self.i * self.integrate_error * 0.1 + self.d * d_error / 0.1

        value = self.__bound(value, self.min_value, self.max_value)
        return value

    def reset(self):
        self.last_error = 0
        self.integrate_error = 0

    def __bound(self, val, min_value, max_value):
        return np.clip(val, min_value, max_value)


class SteerLQR:
    def __init__(self, q, r, dt) -> None:
        # LQR parameter
        self.Q = np.eye(4) * q
        self.R = np.eye(1) * r

        # parameters
        self.dt = dt  # time tick[s]
        self._error = 0.0
        self._error_theta = 0.0

    def solve_DARE(self, A, B, Q, R):
        """
        solve a discrete time_Algebraic Riccati equation (DARE)
        """
        X = Q
        maxiter = 150
        eps = 0.1

        for i in range(maxiter):
            Xn = A.T @ X @ A - A.T @ X @ B @ \
                la.inv(R + B.T @ X @ B) @ B.T @ X @ A + Q
            if (abs(Xn - X)).max() < eps:
                break
            X = Xn

        return Xn

    def dlqr(self, A, B, Q, R):
        """Solve the discrete time lqr controller.
        x[k+1] = A x[k] + B u[k]
        cost = sum x[k].T*Q*x[k] + u[k].T*R*u[k]
        # ref Bertsekas, p.151
        """

        # first, try to solve the ricatti equation
        X = self.solve_DARE(A, B, Q, R)

        # compute the LQR gain
        K = la.inv(B.T @ X @ B + R) @ (B.T @ X @ A)

        eigVals, eigVecs = la.eig(A - B @ K)

        return K, X, eigVals

    def execute(self, state, path):
        ind, e = self.calc_nearest_index(state, path)

        v = state.v
        th_e = pi_2_pi(state.yaw - path.yaw[ind])

        A = np.zeros((4, 4))
        A[0, 0] = 1.0
        A[0, 1] = self.dt
        A[1, 2] = v
        A[2, 2] = 1.0
        A[2, 3] = self.dt

        B = np.zeros((4, 1))
        B[3, 0] = 1.0

        K, _, _ = self.dlqr(A, B, self.Q, self.R)

        x = np.zeros((4, 1))
        x[0, 0] = e
        x[1, 0] = (e - self._error) / self.dt
        x[2, 0] = th_e
        x[3, 0] = (th_e - self._error_theta) / self.dt

        self._error = e
        self._error_theta = th_e

        # ff = math.atan2(L * k, 1)
        fb = pi_2_pi((-K @ x)[0, 0])
        w = fb

        return w

    def calc_nearest_index(self, state, path):
        dx = [state.x - icx for icx in path.x]
        dy = [state.y - icy for icy in path.y]

        d = [idx ** 2 + idy ** 2 for (idx, idy) in zip(dx, dy)]

        mind = min(d)

        ind = d.index(mind)

        mind = math.sqrt(mind)

        dxl = path.x[ind] - state.x
        dyl = path.y[ind] - state.y

        angle = pi_2_pi(path.yaw[ind] - math.atan2(dyl, dxl))
        if angle < 0:
            mind *= -1

        return ind, mind
