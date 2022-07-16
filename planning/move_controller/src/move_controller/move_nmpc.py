from cProfile import label
from time import time
import math

import casadi

import rospy
import numpy as np
from scipy.optimize import NonlinearConstraint
from scipy import optimize

from geometry_msgs.msg import Point, PointStamped


class NonlinearController:
    def __init__(self, frame) -> None:
        self.controller = NonLinearMPC(10, 0.1, [10.0, 10.0, 10.0])
        self.stop_controller = StopNonlinearMPC(
            10, 0.1, [9000.0, 8000.0, 4000.0, 9000.0])
        self._frame = frame

    def execute(self):
        x, y = self._frame.get_robot_location()
        yaw = self._frame.get_robot_yaw()
        speed = self._frame.get_robot_speed()
        angular_speed = self._frame.get_robot_angular_speed()

        target = self._frame.discrete_trajectory.calculate_target_points(
            [x, y], 0.1, 10
        )

        if target:
            result, times_ = self.controller.calculate(
                [x, y, yaw, speed, angular_speed],
                [0.0, 0.5],
                [-1.0, 1.0],
                [-0.5, 0.5],
                [-0.5, 0.5],
                target
            )
        else:
            result, times_ = self.stop_controller.calculate(
                [x, y, yaw, speed, angular_speed],
                [0.0, 0.5],
                [-1.0, 1.0],
                [-0.5, 0.5],
                [-0.5, 0.5],
            )

        self.fill_debug(result, times_)

        return result['v'][1], result['w'][1]

    def generate_stop_target(self):
        pass

    def fill_debug(self, result, times_):
        debug = self._frame.control_debug
        debug.header.stamp = rospy.get_rostime()
        for x, y, v, w, t in zip(
            result['x'], result['y'], result['v'],
            result['w'], times_
        ):
            debug.position_debug.append(Point(x=x, y=y))
            # debug.linear_velocity_debug.append(PointStamped())
            # debug.linear_velocity_debug[-1].header.stamp = rospy.Time.from_sec(t)
            # debug.linear_velocity_debug[-1].point.x = v
            # debug.angular_velocity_debug.append(PointStamped())
            # debug.angular_velocity_debug[-1].header.stamp = rospy.Time.from_sec(t)
            # debug.angular_velocity_debug[-1].point.x = w
            debug.linear_velocity_debug.append(Point(x=t, y=v))
            debug.angular_velocity_debug.append(Point(x=t, y=w))


def pi_2_pi(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi


class NonLinearMPC:
    def __init__(self, frame, horizon, dt, weights, constraints) -> None:
        self._frame = frame
        self._horizon = horizon
        self._dt = dt
        self._w1 = weights[0]
        self._w2 = weights[1]
        self._w3 = weights[2]
        self._w4 = weights[3]

        self.v_upper = constraints['v_upper']
        self.v_lower = constraints['v_lower']

        # Acceleration
        self.a_upper = constraints['a_upper']
        self.a_lower = constraints['a_lower']

        # Angular velocity
        self.w_upper = constraints['w_upper']
        self.w_lower = constraints['w_lower']

        self.last_a = 0.0
        self.last_w = 0.0

    def calc_s(self, path_x, path_y):
        s_len = len(path_x)
        s = [0.0] * s_len

        if s_len < 1:
            return [0.0]

        for i in range(1, s_len):
            dx = path_x[i] - path_x[i - 1]
            dy = path_y[i] - path_y[i - 1]
            s[i] = s[i - 1] + math.hypot(dx, dy)

        return s

    def _referent_speed(self):
        path = self._frame.path
        a_c_ref = 0.3
        epsilon = 10e-6
        k = path.kappa
        v = path.speed

        referent_v = [0.0] * len(v)

        for i in range(len(v)):
            referent_v[i] = min(v[i], np.sqrt(a_c_ref / abs(k[i] + epsilon)))

        # print(f"[MPC] BEFORE: {v=}")
        # print(f"[MPC] AFTER: {referent_v=}")
        return referent_v

    def execute(self):
        ind, e = self._calc_nearest_index()
        path = self._frame.path
        state = self._frame.state
        cropped_x, cropped_y = path.x[ind:], path.y[ind:]
        s = self.calc_s(cropped_x, cropped_y)
        speed = self._referent_speed()
        a, w = self._do_mpc(
            s, cropped_x, cropped_y, speed[ind:],
            state.x, state.y, state.yaw, state.v
        )
        return (state.v + self._dt * a, w)

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

    def _do_mpc(self, s, x, y, speed_profile,
                robot_x, robot_y, robot_yaw, robot_v):
        x_interp = casadi.interpolant('x', 'bspline', [s], x)
        y_interp = casadi.interpolant('y', 'bspline', [s], y)
        speed_interp = casadi.interpolant('speed', 'linear', [s], speed_profile[:len(s)])
        DT = self._dt
        KNOTS = int(self._horizon / DT)
        path_len = len(x)

        # # Speed
        # v_upper = constraints.v_upper
        # v_lower = constraints.v_lower

        # # Acceleration
        # a_upper = constraints.a_upper
        # a_lower = constraints.a_lower

        # # Angular velocity
        # w_upper = constraints.w_upper
        # w_lower = constraints.w_lower

        opti = casadi.Opti()

        cost = 0.0

        X = opti.variable(KNOTS)
        Y = opti.variable(KNOTS)
        ALPHA = opti.variable(KNOTS)
        V = opti.variable(KNOTS)
        S = opti.variable(KNOTS)

        A = opti.variable(KNOTS)
        W = opti.variable(KNOTS)

        opti.subject_to(X[0] == robot_x)
        opti.subject_to(Y[0] == robot_y)
        opti.subject_to(ALPHA[0] == robot_yaw)
        opti.subject_to(V[0] == robot_v)
        opti.subject_to(S[0] == 0.0)
        opti.subject_to(A[0] == self.last_a)
        opti.subject_to(W[0] == self.last_w)
        # opti.subject_to(W[0] == w_0)
        opti.subject_to(opti.bounded(self.v_lower, V[:], self.v_upper))
        opti.subject_to(opti.bounded(self.a_lower, A[:], self.a_upper))
        opti.subject_to(opti.bounded(self.w_lower, W[:], self.w_upper))
        opti.set_initial(A[:], self.last_a)
        opti.set_initial(W[:], self.last_w)

        for i in range(1, KNOTS):
            opti.set_initial(X[i], x[min(i, path_len - 1)])
            opti.set_initial(Y[i], y[min(i, path_len - 1)])
            # opti.set_initial(ALPHA[i], casadi.atan2(y[i] - y[i - 1], x[i] - x[i - 1] + eps))
            opti.set_initial(ALPHA[i], 0.0)
            opti.set_initial(V[i], speed_profile[min(i, path_len - 1)])

            v_target = speed_interp(S[i - 1])

            # opti.subject_to(S[i] == S[i - 1] + V[i] * DT + A[i - 1] * DT ** 2 / 2.0)
            opti.subject_to(S[i] == S[i - 1] + v_target * DT)
            opti.subject_to(X[i] == X[i - 1] + V[i] * casadi.cos(ALPHA[i - 1]) * DT + A[i] * casadi.cos(ALPHA[i - 1]) * DT ** 2 / 2.0)
            opti.subject_to(Y[i] == Y[i - 1] + V[i] * casadi.sin(ALPHA[i - 1]) * DT + A[i] * casadi.sin(ALPHA[i - 1]) * DT ** 2 / 2.0)
            opti.subject_to(ALPHA[i] == ALPHA[i - 1] + W[i] * DT)
            opti.subject_to(V[i] == V[i - 1] + A[i] * DT)

        cost_1 = 0.0
        cost_2 = 0.0
        cost_3 = 0.0
        cost_4 = 0.0

        for i in range(1, KNOTS):
            x_target = x_interp(S[i])
            y_target = y_interp(S[i])
            v_target = speed_interp(S[i])

            cost_1 += self._w1 * ((X[i] - x_target)**2 + (Y[i] - y_target)**2)
            cost_2 += self._w2 * (V[i] - v_target)**2
            cost_3 += self._w3 * (A[i] - A[i - 1])**2
            cost_4 += self._w4 * (W[i] - W[i - 1])**2

        cost += cost_1 + cost_2 + cost_3 + cost_4

        opti.minimize(cost)
        opts = {'ipopt.print_level': 0, 'print_time': 0, 'ipopt.sb': 'yes'}
        opti.solver('ipopt', opts)

        a_res, v_res = 0.0, 0.0
        try:
            sol = opti.solve()
            a_res = sol.value(A[1])
            v_res = sol.value(W[1])

            self.last_a = a_res
            self.last_w = v_res
        except:
            a_res = opti.debug.value(A[1])
            v_res = opti.debug.value(W[1])

            self.last_a = a_res
            self.last_w = v_res
        return a_res, v_res


class StopNonlinearMPC:
    def __init__(self, knots, dt, weights) -> None:
        self._knots = knots
        self._dt = dt
        self._w1 = weights[0]
        self._w2 = weights[1]
        self._w3 = weights[2]
        self._w4 = weights[3]

    def calculate(self, init_x, v_bounds, w_bounds, dv_bounds, dw_bounds):
        opti = casadi.Opti()

        cost = 0.0

        X = opti.variable(self._knots)
        Y = opti.variable(self._knots)
        ALPHA = opti.variable(self._knots)
        V = opti.variable(self._knots)
        W = opti.variable(self._knots)

        opti.subject_to(X[0] == init_x[0])
        opti.subject_to(Y[0] == init_x[1])
        opti.subject_to(ALPHA[0] == init_x[2])
        opti.subject_to(V[0] == init_x[3])
        opti.subject_to(W[0] == init_x[4])
        opti.subject_to(opti.bounded(v_bounds[0], V[:], v_bounds[1]))
        opti.subject_to(opti.bounded(w_bounds[0], W[:], w_bounds[1]))

        for i in range(1, self._knots):
            opti.subject_to(opti.bounded(
                dv_bounds[0], (V[i] - V[i - 1]) / self._dt, dv_bounds[1]))
            opti.subject_to(opti.bounded(
                dw_bounds[0], (W[i] - W[i - 1]) / self._dt, dw_bounds[1]))

            opti.subject_to(X[i] == X[i - 1] + V[i] *
                            casadi.cos(ALPHA[i - 1]) * self._dt)
            opti.subject_to(Y[i] == Y[i - 1] + V[i] *
                            casadi.sin(ALPHA[i - 1]) * self._dt)
            opti.subject_to(ALPHA[i] == ALPHA[i - 1] + W[i] * self._dt)

            cost_1 = self._w1 * ((X[i] - X[i - 1])**2 + (Y[i] - Y[i - 1])**2)
            cost_2 = self._w2 * (V[i])**2
            cost_3 = self._w3 * (V[i] - V[i - 1])**2

            cost += cost_1 + cost_2 + cost_3

        opti.minimize(cost)
        opts = {'ipopt.print_level': 0, 'print_time': 0, 'ipopt.sb': 'yes'}
        opti.solver('ipopt', opts)

        sol = opti.solve()

        times = [0.0] * self._knots

        result_debug = {
            "x": [0.0] * self._knots,
            "y": [0.0] * self._knots,
            "v": [0.0] * self._knots,
            "w": [0.0] * self._knots
        }

        for i in range(self._knots):
            result_debug['x'][i] = sol.value(X[i])
            result_debug['y'][i] = sol.value(Y[i])
            result_debug['v'][i] = sol.value(V[i])
            result_debug['w'][i] = sol.value(W[i])
            times[i] = i * self._dt

        return result_debug, times


def test_main():
    target = [
        [0.0, 1.0, 0.5],
        [0.05, 1.0, 0.5],
        [0.10, 1.0, 0.5],
        [0.15, 1.0, 0.5],
        [0.20, 1.0, 0.5],
        [0.25, 1.0, 0.5],
        [0.30, 1.0, 0.5],
        [0.35, 1.0, 0.5],
        [0.40, 1.0, 0.5],
        [0.45, 1.0, 0.5],
    ]
    controller = NonLinearMPC(
        2, 0.1, [1.0, 10.0, 1.0, 1.0],
        {
            "v_lower": 0.0,
            "v_upper": 2.0,
            "a_lower": -1.0,
            "a_upper": 1.0,
            "w_upper": np.deg2rad(20),
            "w_lower": -np.deg2rad(20)
        })
    debug, times = controller.calculate(
        [0.0, 0.0,
         0.1, 0.0, 0.0],
        [0.0, 1.0],
        [-1.0, 1.0],
        [-1.0, 1.0],
        [-0.5, 0.5],
        target
    )

    import matplotlib.pyplot as plt
    # fig = plt.figure()
    # ax = plt.axes()
    fig, axs = plt.subplots(3)
    fig.suptitle('Vertically stacked subplots')
    # axs[0].plot(x, y)
    # axs[1].plot(x, -y)
    target_x, target_y = [], []
    for x, y, _ in target:
        target_x.append(x)
        target_y.append(y)

    axs[0].plot(debug['x'], debug['y'], 'o', label='robot')
    axs[0].plot(target_x, target_y, 'o', label='robot')
    plt.legend()
    # ax.plot(times, debug['x'], label='dx')
    # ax.plot(times, debug['x'], label='ddx')

    # plt.legend()
    # plt.show()
    axs[1].plot(times, debug['v'], label='linear speed')
    plt.legend()
    axs[2].plot(times, debug['w'], label='angular speed')
    plt.legend()
    plt.show()


def test_stop():
    stop_controller = StopNonlinearMPC(
        15, 0.1, [9000.0, 8000.0, 4000.0, 9000.0])
    debug, times = stop_controller.calculate(
        [0.0, 0.0, 0.0, 1.0, 0.05],
        [0.0, 1.0],
        [-5.0, 5.0],
        [-5.0, 5.0],
        [-5.0, 5.0]
    )

    import matplotlib.pyplot as plt
    # fig = plt.figure()
    # ax = plt.axes()
    fig, axs = plt.subplots(3)
    fig.suptitle('Vertically stacked subplots')
    # axs[0].plot(x, y)
    # axs[1].plot(x, -y)
    axs[0].plot(debug['x'], debug['y'], 'o', label='robot')
    plt.legend()
    # ax.plot(times, debug['x'], label='dx')
    # ax.plot(times, debug['x'], label='ddx')

    # plt.legend()
    # plt.show()
    axs[1].plot(times, debug['v'], label='linear speed')
    plt.legend()
    axs[2].plot(times, debug['w'], label='angular speed')
    plt.legend()
    plt.show()


if __name__ == "__main__":
    test_main()
