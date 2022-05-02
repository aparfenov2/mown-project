from cProfile import label
from time import time

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


class NonLinearMPC:
    def __init__(self, knots, dt, weights) -> None:
        self._knots = knots
        self._dt = dt
        self._w1 = weights[0]
        self._w2 = weights[1]
        self._w3 = weights[2]

    def calculate(self, init_x, v_bounds, w_bounds, dv_bounds, dw_bounds, target):
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


        # opti.set_initial(S[:], s_bounds[1])

        for i in range(1, self._knots):
            x_target = target[i - 1][0]
            y_target = target[i - 1][1]
            v_target = target[i - 1][2]

            opti.set_initial(X[i], x_target)
            opti.set_initial(Y[i], y_target)
            opti.set_initial(ALPHA[i], init_x[2])
            opti.set_initial(V[i], v_target)

            # opti.subject_to(opti.bounded(
            #     dv_bounds[0], (V[i] - V[i - 1]) / self._dt, dv_bounds[1]))
            # opti.subject_to(opti.bounded(
            #     dw_bounds[0], (W[i] - W[i - 1]) / self._dt, dw_bounds[1]))

            opti.subject_to(X[i] == X[i - 1] + V[i] *
                            casadi.cos(ALPHA[i - 1]) * self._dt)
            opti.subject_to(Y[i] == Y[i - 1] + V[i] *
                            casadi.sin(ALPHA[i - 1]) * self._dt)
            opti.subject_to(ALPHA[i] == ALPHA[i - 1] + W[i] * self._dt)

            cost_1 = self._w1 * ((X[i] - x_target)**2 + (Y[i] - y_target)**2)
            cost_2 = self._w2 * (V[i] - v_target)**2
            cost_3 = self._w3 * ((V[i] - V[i - 1])**2 + (W[i] - W[i - 1])**2)

            cost += cost_1 + cost_2

        opti.minimize(cost)
        opts = {'ipopt.print_level': 0, 'print_time': 0, 'ipopt.sb': 'yes'}
        opti.solver('ipopt', opts)

        times = [0.0] * self._knots

        result_debug = {
            "x": [0.0] * self._knots,
            "y": [0.0] * self._knots,
            "v": [0.0] * self._knots,
            "w": [0.0] * self._knots
        }
        try:
            sol = opti.solve()

            for i in range(self._knots):
                result_debug['x'][i] = sol.value(X[i])
                result_debug['y'][i] = sol.value(Y[i])
                result_debug['v'][i] = sol.value(V[i])
                result_debug['w'][i] = sol.value(W[i])
                times[i] = i * self._dt
        except:
            rospy.logwarn(f"{init_x=}\n{target=}")
            for i in range(self._knots):
                result_debug['x'][i] = opti.debug.value(X[i])
                result_debug['y'][i] = opti.debug.value(Y[i])
                result_debug['v'][i] = opti.debug.value(V[i])
                result_debug['w'][i] = opti.debug.value(W[i])
                times[i] = i * self._dt

        return result_debug, times


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
    controller = NonLinearMPC(10, 0.1, [1.0, 10.0, 1.0])
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
