"""

Path tracking simulation with LQR steering control and PID speed control.

author Atsushi Sakai (@Atsushi_twi)

"""
import scipy.linalg as la
import matplotlib.pyplot as plt
import math
import numpy as np
import sys
import os

import rospy


def pi_2_pi(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi


def solve_dare(A, B, Q, R):
    """
    solve a discrete time_Algebraic Riccati equation (DARE)
    """
    X = Q
    maxiter = 150
    eps = 0.01

    for i in range(maxiter):
        Xn = A.T @ X @ A - A.T @ X @ B @ \
            la.inv(R + B.T @ X @ B) @ B.T @ X @ A + Q
        if (abs(Xn - X)).max() < eps:
            break
        X = Xn

    return Xn


def dlqr(A, B, Q, R):
    """Solve the discrete time lqr controller.
    x[k+1] = A x[k] + B u[k]
    cost = sum x[k].T*Q*x[k] + u[k].T*R*u[k]
    # ref Bertsekas, p.151
    """

    # first, try to solve the ricatti equation
    X = solve_DARE(A, B, Q, R)

    # compute the LQR gain
    K = la.inv(B.T @ X @ B + R) @ (B.T @ X @ A)

    eigVals, eigVecs = la.eig(A - B @ K)

    return K, X, eigVals


def lqr_steering_control(state, cx, cy, cyaw, ck, pe, pth_e):
    ind, e = calc_nearest_index(state, cx, cy, cyaw)

    k = ck[ind]
    v = state.v
    th_e = pi_2_pi(state.yaw - cyaw[ind])

    A = np.zeros((4, 4))
    A[0, 0] = 1.0
    A[0, 1] = dt
    A[1, 2] = v
    A[2, 2] = 1.0
    A[2, 3] = dt
    # print(A)

    B = np.zeros((4, 1))
    B[3, 0] = v / L

    K, _, _ = dlqr(A, B, Q, R)

    x = np.zeros((4, 1))

    x[0, 0] = e
    x[1, 0] = (e - pe) / dt
    x[2, 0] = th_e
    x[3, 0] = (th_e - pth_e) / dt

    ff = math.atan2(L * k, 1)
    fb = pi_2_pi((-K @ x)[0, 0])

    delta = ff + fb

    return delta, ind, e, th_e


def calc_nearest_index(state, cx, cy, cyaw):
    dx = [state.x - icx for icx in cx]
    dy = [state.y - icy for icy in cy]

    d = [idx ** 2 + idy ** 2 for (idx, idy) in zip(dx, dy)]

    mind = min(d)

    ind = d.index(mind)

    mind = math.sqrt(mind)

    dxl = cx[ind] - state.x
    dyl = cy[ind] - state.y

    angle = pi_2_pi(cyaw[ind] - math.atan2(dyl, dxl))
    if angle < 0:
        mind *= -1

    return ind, mind


def closed_loop_prediction(cx, cy, cyaw, ck, speed_profile, goal):
    T = 500.0  # max simulation time
    goal_dis = 0.3
    stop_speed = 0.05

    state = State(x=-0.0, y=-0.0, yaw=0.0, v=0.0)

    time = 0.0
    x = [state.x]
    y = [state.y]
    yaw = [state.yaw]
    v = [state.v]
    t = [0.0]

    e, e_th = 0.0, 0.0

    while T >= time:
        dl, target_ind, e, e_th = lqr_steering_control(
            state, cx, cy, cyaw, ck, e, e_th)

        ai = PIDControl(speed_profile[target_ind], state.v)
        state = update(state, ai, dl)

        if abs(state.v) <= stop_speed:
            target_ind += 1

        time = time + dt

        # check goal
        dx = state.x - goal[0]
        dy = state.y - goal[1]
        if math.hypot(dx, dy) <= goal_dis:
            print("Goal")
            break

        x.append(state.x)
        y.append(state.y)
        yaw.append(state.yaw)
        v.append(state.v)
        t.append(time)

        if target_ind % 1 == 0 and show_animation:
            plt.cla()
            # for stopping simulation with the esc key.
            plt.gcf().canvas.mpl_connect('key_release_event',
                    lambda event: [exit(0) if event.key == 'escape' else None])
            plt.plot(cx, cy, "-r", label="course")
            plt.plot(x, y, "ob", label="trajectory")
            plt.plot(cx[target_ind], cy[target_ind], "xg", label="target")
            plt.axis("equal")
            plt.grid(True)
            plt.title("speed[km/h]:" + str(round(state.v * 3.6, 2))
                      + ",target index:" + str(target_ind))
            plt.pause(0.0001)

    return t, x, y, yaw, v


def calc_speed_profile(cx, cy, cyaw, target_speed):
    speed_profile = [target_speed] * len(cx)

    direction = 1

    # Set stop point
    for i in range(len(cx) - 1):
        dyaw = abs(cyaw[i + 1] - cyaw[i])
        switch = math.pi / 4.0 <= dyaw < math.pi / 2.0

        if switch:
            direction *= -1

        if direction != 1:
            speed_profile[i] = -target_speed
        else:
            speed_profile[i] = target_speed

        if switch:
            speed_profile[i] = 0.0

    speed_profile[-1] = 0.0

    return speed_profile


class LQRController:
    def __init__(self, frame, dt) -> None:
        self._frame = frame
        self._error = 0.0
        self._error_theta = 0.0
        self._dt = dt
        self._last_v = 0.0

        self._last_stamp = None

        self._Q = np.eye(5) * 1.0
        self._R = np.eye(2) * 100.0

    def execute(self):
        # cur_stamp = rospy.get_time()
        # if self._last_stamp is None:
        #     self._dt = 0.1
        # else:
        #     self._dt = cur_stamp - self._last_stamp

        # self._last_stamp = cur_stamp
        self._dt = 0.1
        v, w = self._lqr_control()
        return (v, w)

    def _dlqr(self, A, B):
        """
        Solve the discrete time lqr controller.
        x[k+1] = A x[k] + B u[k]
        cost = sum x[k].T*Q*x[k] + u[k].T*R*u[k]
        # ref Bertsekas, p.151
        """
        # first, try to solve the ricatti equation
        Q = self._Q
        R = self._R
        X = solve_dare(A, B, Q, R)

        # compute the LQR gain
        K = la.inv(B.T @ X @ B + R) @ (B.T @ X @ A)

        eigVals, eigVecs = la.eig(A - B @ K)

        return K, X, eigVals

    def _lqr_control(self):
        state = self._frame.state
        path = self._frame.path
        ind, e = self._calc_nearest_index()

        tv = path.speed[ind]

        v = state.v
        th_e = pi_2_pi(state.yaw - path.yaw[ind])

        # A = [1.0, dt, 0.0, 0.0, 0.0
        #      0.0, 0.0, v, 0.0, 0.0]
        #      0.0, 0.0, 1.0, dt, 0.0]
        #      0.0, 0.0, 0.0, 0.0, 0.0]
        #      0.0, 0.0, 0.0, 0.0, 1.0]
        A = np.zeros((5, 5))
        A[0, 0] = 1.0
        A[0, 1] = self._dt
        A[1, 2] = v
        A[2, 2] = 1.0
        A[2, 3] = self._dt
        A[4, 4] = 1.0

        # B = [0.0, 0.0
        #      0.0, 0.0
        #      0.0, 0.0
        #      1.0, 0.0
        #      0.0, dt ]
        B = np.zeros((5, 2))
        B[3, 0] = 1.0
        B[4, 1] = self._dt

        K, _, _ = self._dlqr(A, B)

        # state vector
        # x = [e, dot_e, th_e, dot_th_e, delta_v]
        # e: lateral distance to the path
        # dot_e: derivative of e
        # th_e: angle difference to the path
        # dot_th_e: derivative of th_e
        # delta_v: difference between current speed and target speed
        x = np.zeros((5, 1))
        x[0, 0] = e
        x[1, 0] = (e - self._error) / self._dt
        x[2, 0] = th_e
        x[3, 0] = (th_e - self._error_theta) / self._dt
        x[4, 0] = v - tv

        # input vector
        # u = [delta, accel]
        # delta: steering angle
        # accel: acceleration

        self._error = e
        self._error_theta = th_e

        ustar = -K @ x

        # calc steering input
        # ff = math.atan2(L * k, 1)  # feedforward steering angle
        # fb = pi_2_pi(ustar[0, 0])  # feedback steering angle
        # delta = ff + fb
        w = ustar[0, 0] + (path.yaw[ind] - state.yaw) / self._dt

        # calc accel input
        accel = ustar[1, 0]
        print(f"Error: {e}, robot v: {v}, target v: {tv}, index: {ind}, accel: {accel}, w: {w}")

        return v + accel * self._dt, w
        # return w, accel

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
