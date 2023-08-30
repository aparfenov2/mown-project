from cProfile import label
from time import time

import casadi

import rospy
import numpy as np
from geometry_msgs.msg import Point, PointStamped


class FollowNMPC:
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

        for i in range(1, self._knots):
            x_star = X[i] / (target[1] + 1)

            opti.subject_to(opti.bounded(
                dv_bounds[0], (V[i] - V[i - 1]) / self._dt, dv_bounds[1]))
            opti.subject_to(opti.bounded(
                dw_bounds[0], (W[i] - W[i - 1]) / self._dt, dw_bounds[1]))

            opti.subject_to(X[i] == X[i - 1] + V[i] *
                            casadi.cos(ALPHA[i - 1]) * self._dt)
            opti.subject_to(Y[i] == Y[i - 1] + V[i] *
                            casadi.sin(ALPHA[i - 1]) * self._dt)
            opti.subject_to(ALPHA[i] == ALPHA[i - 1] + W[i] * self._dt)

            cost_1 = self._w1 * ((X[i] - x_star)**2 + (Y[i] - (target[0] + ))**2)
            cost_2 = self._w2 * (V[i] - v_target)**2
            cost_3 = self._w3 * ((V[i] - V[i - 1])**2 + (W[i] - W[i - 1])**2)

            cost += cost_1 + cost_3

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
        [-0.0019220522170034383, -0.006049293353311313, 0.0014598630663463195],
        [-0.001927207600632259, -0.006064836825374676, 0.0064598631663333695],
        [-0.0019837705676531868, -0.006235374065861631, 0.05645986416620173],
        [-0.0030492467911768712, -0.009504971199735847, 0.10645986516604808],
        [-0.010952863092954535, -0.03775918280408899, 0.15645986616586546],
        [0.00016336173663332064, -0.050584708388158525, 0.20645986716564338],
        [0.03230584426858592, -0.060165961687801035, 0.25645986816536476],
        [0.053764103135073446, -0.06302696450823742, 0.3064598691649989],
        [0.09322218113220454, -0.0667935853627846, 0.35645987016448144],
        [0.1331035854218894, -0.06938161919267184, 0.40645987116363086]
    ]
    controller = NonLinearMPC(10, 0.1, [100.0, 10.0, 10.0])
    debug, times = controller.calculate(
        [-0.0019220522170034383, -0.006049293353311313,
         1.8 * np.pi, 0.1, -0.01],
        [0.0, 0.5],
        [-0.8, 0.8],
        [-0.5, 0.5],
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
