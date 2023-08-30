from cProfile import label
from math import fabs
from select import select
from time import time

import casadi
import rospy
import numpy as np
from scipy.optimize import NonlinearConstraint
from scipy import optimize
from task_behavior_engine.tree import Node, NodeStatus

from geometry_msgs.msg import Point
from enginx_msgs.msg import Route, PointWithSpeed, SpeedGeneratorDebug


class SpeedGeneratorNode(Node):
    def __init__(self, name, frame, *args, **kwargs):
        super(SpeedGeneratorNode, self).__init__(name=name,
                                                 run_cb=self.run,
                                                 *args, **kwargs)
        self._frame = frame
        self._smoother = NonlinaerSmootherCasadi(20, 0.1, [100.0, 1.0, 0.1, 0.1])
        self._target_speed = 0.5

    def run(self, nodedata):
        current_v = max(self._frame.linear_speed, 0.0)
        current_dv = self._frame.linear_acceleration
        max_s = self._frame.discrete_trajectory.get_max_s()

        s, ds, _, times_ = self._smoother.smooth(
            [0.0, current_v, current_dv],
            [0.0, max_s],
            [0.0, self._target_speed],
            [-0.5, 0.5],
            [-5.0, 5.0],
            [max_s, self._target_speed]
        )

        route = Route()
        route.header.stamp = self._frame.get_localization().header.stamp
        for s_i, ds_i, t_i in zip(s, ds, times_):
            point = self._frame.discrete_trajectory.get_point_at_s(s_i)
            pws = PointWithSpeed()
            pws.x = point[0]
            pws.y = point[1]
            pws.speed = ds_i
            pws.d_time = t_i
            route.route.append(pws)
        self._frame.set_trajectory(route)

        self._fill_debug(s, ds)

        return NodeStatus(NodeStatus.SUCCESS)

    def _fill_debug(self, s, ds):
        self._frame.planning_debug.speed_generator_debug = SpeedGeneratorDebug()
        debug_points = self._frame.planning_debug.speed_generator_debug.points
        for s_i, ds_i in zip(s, ds):
            point = Point(x=s_i, y=ds_i)
            debug_points.append(point)


class SimpleSpeedGenerator(Node):
    def __init__(self, name, frame, *args, **kwargs):
        super(SimpleSpeedGenerator, self).__init__(name=name,
                                                   run_cb=self.run,
                                                   *args, **kwargs)
        self._frame = frame
        self._target_speed = 0.5

    def run(self, nodedata):
        if self._frame.get_path_done():
            return NodeStatus(NodeStatus.SUCCESS)

        route = Route()
        route.header.stamp = self._frame.get_localization().header.stamp
        for x, y in self._frame.discrete_trajectory.path:
            pws = PointWithSpeed()
            pws.x = x
            pws.y = y
            pws.speed = self._target_speed
            pws.d_time = 0.0
            route.route.append(pws)

        if len(route.route) > 0:
            path_points = min(len(route.route), 20)
            for i in range(1, path_points + 1):
                route.route[-i].speed = min(i * 0.05, route.route[-i].speed)

        self._frame.set_trajectory(route)

        return NodeStatus(NodeStatus.SUCCESS)


class NonlinearSmootherScipy:
    def __init__(self, knots, dt) -> None:
        self._knots = knots
        self._dt = dt

    def cost_function(self, x, referents):
        cost = 0
        s_ref = referents[0]
        ds_ref = referents[1]

        for i in range(1, self._knots):
            s = x[i]
            ds = x[self._knots + i]
            dds = x[self._knots * 2 + i]

            cost += (s - s_ref) ** 2
            cost += (ds - ds_ref) ** 2
            cost += (dds) ** 2
        return cost

    def constraints(self, x):

        con = [0] * (2 * self._knots)
        con = np.array(con)

        for i in range(1, self._knots):
            s = x[i]
            ds = x[self._knots + i]

            prev_s = x[i]
            prev_ds = x[self._knots + i]
            prev_dds = x[self._knots * 2 + i]

            con[i] = s - (prev_s + prev_ds * self._dt + prev_dds * 0.5 * self._dt ** 2)
            con[self._knots + i] = ds - (prev_ds + prev_dds * self._dt)
        return con

    def smooth(self, s_init, ds_init, dds_init, s_max, s_min, ds_max, ds_min, dds_max, dds_min, s_target, ds_target):
        bounds = []
        bounds += [(s_min, s_max) for _ in range(self._knots)]
        bounds += [(ds_min, ds_max) for _ in range(self._knots)]
        bounds += [(dds_min, dds_max) for _ in range(self._knots)]

        bounds[0] = [s_init, s_init]
        bounds[self._knots] = [ds_init, ds_init]
        bounds[2*self._knots] = [dds_init, dds_init]

        nlc = NonlinearConstraint(
            self.constraints,
            np.array([0.0] * (self._knots * 2)),
            np.array([0.0] * (self._knots * 2)))

        x0 = [s_init] * self._knots + [ds_init] * self._knots + [dds_init] * self._knots

        solution = optimize.minimize(self.cost_function,
                                     x0=x0,
                                     bounds=bounds,
                                     args=([s_target, ds_target]),
                                     constraints=nlc,
                                     method='SLSQP',
                                     tol=1e-5,
                                     options={'eps': 0.01, 'disp': False})
        if solution.success:
            self.last_v0 = solution.x[0]
            self.last_w0 = solution.x[self._knots]
            return self.last_v0, self.last_w0

        return None, None


class NonlinaerSmootherCasadi:
    def __init__(self, knots, dt, w) -> None:
        self._knots = knots
        self._dt = dt
        self._w1 = w[0]
        self._w2 = w[1]
        self._w3 = w[2]
        self._w4 = w[3]

    def smooth_(self, s_init, s_bounds, ds_bounds, dds_bounds, ddds_bounds, target):
        opti = casadi.Opti()

        var_s = [None] * self._knots
        var_ds = [None] * self._knots
        var_dds = [None] * self._knots

        s = opti.variable()
        ds = opti.variable()
        dds = opti.variable()

        cost = 0.0

        opti.subject_to(s == s_init[0])
        opti.subject_to(ds == s_init[1])
        opti.subject_to(dds == s_init[2])

        prev_s = s
        prev_ds = ds
        prev_dds = dds

        var_s[0] = prev_s
        var_ds[0] = prev_ds
        var_dds[0] = prev_dds

        for i in range(1, self._knots):
            s_i = opti.variable()
            ds_i = opti.variable()
            dds_i = opti.variable()

            opti.subject_to(opti.bounded(s_bounds[0], s_i, s_bounds[1]))
            # opti.subject_to(opti.bounded(ds_min, ds_i, ds_max))
            opti.subject_to(opti.bounded(ds_bounds[0], ds_i, ds_bounds[1]))
            
            opti.subject_to(opti.bounded(dds_bounds[0], dds_i, dds_bounds[1]))
            # opti.subject_to(opti.bounded(ddds_min, (dds_i - prev_ds) / self._dt, ddds_max))
            opti.subject_to(prev_ds + dds_i * self._dt == ds_i)
            opti.subject_to(prev_s + prev_ds * self._dt + dds_i * 0.5 * (self._dt ** 2) == s_i)


            cost_ds = self._w1 * (ds_i - target[1])**2
            # cost_bounds = self._w2 * casadi.if_else(var_s[i] >= target[0] - 0.01, 100 * (var_ds[i])**2, 0.0) for i in range(self._knots)])
            # cost_dds = self._w3 * sum([casadi.if_else(var_s[i] <= target[0], 0.0, (var_dds[i]) ** 2) for i in range(self._knots)])
            # cost_smooth = self._w4 * sum([(var_dds[i] - var_dds[i-1]) ** 2 for i in range(1, self._knots)])

            cost += cost_ds

            prev_s = s_i
            prev_ds = ds_i
            prev_dds = dds_i

            var_s[i] = prev_s
            var_ds[i] = prev_ds
            var_dds[i] = prev_dds

        # opti.minimize(cost)
        # cost_ds = self._w1 * sum([casadi.if_else(var_s[i] <= target[0], (var_ds[i] - 2.0)**2, (10*var_ds[i])**2) for i in range(self._knots)])
        # cost_ds = self._w1 * sum([(0.1*var_ds[i] - 0.08)**2 for i in range(self._knots)])
        # cost_bounds = self._w2 * sum([casadi.if_else(var_s[i] >= target[0] - 0.01, 100 * (var_ds[i])**2, 0.0) for i in range(self._knots)])
        # cost_dds = self._w3 * sum([casadi.if_else(var_s[i] <= target[0], 0.0, (var_dds[i]) ** 2) for i in range(self._knots)])
        # cost_smooth = self._w4 * sum([(var_dds[i] - var_dds[i-1]) ** 2 for i in range(1, self._knots)])
        opti.minimize(cost)

        opts = {'ipopt.print_level': 0, 'print_time': 0, 'ipopt.tol': 10e-7, 'ipopt.sb': 'yes'}
        opti.solver('ipopt', opts)

        sol = opti.solve()
        x_results = [0.0] * self._knots
        dx_results = [0.0] * self._knots
        ddx_results = [0.0] * self._knots

        times = [0.0] * self._knots

        for i in range(self._knots):
            x_results[i] = sol.value(var_s[i])
            dx_results[i] = sol.value(var_ds[i])
            ddx_results[i] = sol.value(var_dds[i])
            times[i] = i * self._dt

        return x_results, dx_results, ddx_results, times

    def smooth(self, s_init, s_bounds, ds_bounds, dds_bounds, ddds_bounds, target, *args, **kwargs):
        opti = casadi.Opti()

        S = opti.variable(self._knots)
        DS = opti.variable(self._knots)
        DDS = opti.variable(self._knots)

        # var_s = [None] * self._knots
        # var_ds = [None] * self._knots
        # var_dds = [None] * self._knots

        cost = 0.0

        opti.subject_to(S[0] == s_init[0])
        opti.subject_to(DS[0] == s_init[1])
        opti.subject_to(DDS[0] == s_init[2])

        # opti.subject_to(opti.bounded(s_bounds[0], X[s_id, :], s_bounds[1]))
        # opti.subject_to(opti.bounded(ds_bounds[0], X[ds_id, :], ds_bounds[1]))
        opti.subject_to(opti.bounded(dds_bounds[0], DDS[:], dds_bounds[1]))
        opti.subject_to(S[:] >= s_bounds[0])
        opti.subject_to(S[:] <= s_bounds[1])
        opti.subject_to(DS[:] >= ds_bounds[0])
        opti.subject_to(DS[:] <= ds_bounds[1])
        # opti.subject_to(DDS[:] >= dds_bounds[0])
        # opti.subject_to(DDS[:] <= dds_bounds[1])

        for i in range(1, self._knots):
            # s_i = opti.variable()
            # ds_i = opti.variable()
            # dds_i = opti.variable()

            dds_next = (DS[i] - DS[i - 1]) / self._dt
            opti.subject_to(dds_next == DDS[i])
            # ds_next = DS[i - 1] + DDS[i] * self._dt
            # opti.subject_to(ds_next == DS[i])
            # s_next = S[i - 1] + DS[i - 1] * self._dt + 0.5 * DDS[i] * (self._dt ** 2)
            # opti.subject_to(s_next == S[i])
            opti.subject_to(S[i - 1] + DS[i] * self._dt == S[i])

            # b_cost_1 = 0.1 * casadi.if_else(DDS[i] >= dds_bounds[1], (DDS[i])**2, 0.0)
            # opti.subject_to(S[i] >= s_bounds[0])
            # opti.subject_to(S[i] <= s_bounds[1])
            # opti.subject_to(DS[i] >= ds_bounds[0])
            # opti.subject_to(DS[i] <= ds_bounds[1])
            # opti.subject_to(DDS[i] >= dds_bounds[0])
            # opti.subject_to(DDS[i] <= dds_bounds[1])

            # prev_s = s_i
            # prev_ds = ds_i
            # prev_dds = dds_i

            # var_s[i] = prev_s
            # var_ds[i] = prev_ds
            # var_dds[i] = prev_dds
            cost_s = self._w1 * (S[i] - s_bounds[1])**2
            # cost_1 = self._w2 * (DS[i] - target[1])**2
            # cost_1 = self._w2 * casadi.if_else(DS[i] <= target[1], -DS[i], (DS[i])**2)
            # cost_ds_1 = self._w3 * (DS[i] - DS[i - 1]) ** 2
            cost_ds_2 = self._w2 * (DDS[i] - DDS[i - 1]) ** 2
            # cost_1 = self._w1 * casadi.if_else((DDS[i] - target[1]) ** 2 > 0.0, (DDS[i])**2, 0.0)
            # cost_2 = -self._w3 * (DDS[i])**2
            # cost_3 = self._w3 * (DDS[i] - DDS[i-1])**2
            # cost_2 = self._w2 * (DS[i] - DS[i-1])**2
            # cost_3 = self._w3 * casadi.if_else(S[i] >= target[0], (DS[i])**2, 0.0)
            cost_4 = self._w4 * casadi.if_else(S[i] >= target[0], (DS[i])**2, 0.0)
            cost += cost_s + cost_ds_2 + cost_4
            # cost += cost_1 + cost_2

        opti.set_initial(S[:], s_bounds[1])
        opti.set_initial(DS[:], ds_bounds[1])
        opti.set_initial(DDS[:], dds_bounds[1])

        # opti.minimize(cost)
        # cost_ds = self._w1 * sum([casadi.if_else(var_s[i] <= target[0], (var_ds[i] - 2.0)**2, (10*var_ds[i])**2) for i in range(self._knots)])
        # cost_ds = self._w1 * sum([(0.1*var_ds[i] - 0.08)**2 for i in range(self._knots)])
        # cost_bounds = self._w2 * sum([casadi.if_else(var_s[i] >= target[0] - 0.01, 100 * (var_ds[i])**2, 0.0) for i in range(self._knots)])
        # cost_dds = self._w3 * sum([casadi.if_else(var_s[i] <= target[0], 0.0, (var_dds[i]) ** 2) for i in range(self._knots)])
        # cost_smooth = self._w4 * sum([(var_dds[i] - var_dds[i-1]) ** 2 for i in range(1, self._knots)])
        # opti.minimize(cost_ds)


        opti.minimize(cost)
        # opts = {'ipopt.print_level': 0, 'print_time': 0, 'ipopt.tol': 10e-8, 'ipopt.max_iter': 20, 'ipopt.sb': 'yes'}
        opts = {
            'ipopt.print_level': 0,
            'print_time': 0,
            'ipopt.fixed_variable_treatment': "make_parameter",
            # 'ipopt.s_max': 10,
            'ipopt.sb': 'yes',
            # 'ipopt.bound_relax_factor': 0.1,
            # 'ipopt.linear_solver': "ma77",
            # 'ipopt.warm_start_init_point': 'yes',
            # 'ipopt.warm_start_bound_push': 1e-9,
            # 'ipopt.warm_start_bound_frac': 1e-9,
            # 'ipopt.warm_start_slack_bound_frac': 1e-9,
            # 'ipopt.warm_start_slack_bound_push': 1e-9,
            # 'ipopt.warm_start_mult_bound_push': 1e-9
        }
        # opts = {'ipopt.print_level': 0, 'print_time': 0, 'ipopt.sb': 'yes'}
        opti.solver('ipopt', opts)
        # opti.solver('ipopt')

        x_results = [0.0] * self._knots
        dx_results = [0.0] * self._knots
        ddx_results = [0.0] * self._knots
        times = [0.0] * self._knots

        try:
            sol = opti.solve()


            for i in range(self._knots):
                x_results[i] = sol.value(S[i])
                dx_results[i] = sol.value(DS[i])
                ddx_results[i] = sol.value(DDS[i])
                times[i] = i * self._dt
        except Exception as e:
            print(e)
            print("HELP")
            for i in range(self._knots):
                x_results[i] = opti.debug.value(S[i])
                dx_results[i] = opti.debug.value(DS[i])
                ddx_results[i] = opti.debug.value(DDS[i])
                times[i] = i * self._dt

        return x_results, dx_results, ddx_results, times


def test_speed_generator():
    smoother = NonlinaerSmootherCasadi(41, 0.1, [10.0, 10.0, 10.0, 10.0])
    max_s = 20.0
    _target_speed = 0.5
    xs, dxs, ddxs, times = smoother.smooth(
        [0.4, 0.2, 0.1],
        [0.0, max_s],
        [0.0, _target_speed], [-0.5, 0.5], [-5.0, 5.0],
        [max_s, _target_speed]
    )

    import matplotlib.pyplot as plt
    fig = plt.figure()
    ax = plt.axes()
    print(f"{dxs=}")
    ax.plot(times, xs, label='x')
    ax.plot(times, dxs, label='dx')
    ax.plot(times, ddxs, label='ddx')

    plt.legend()
    plt.show()


def test_curve(a, b, u, L):
    # //Finds an unknown point distance L along a quadratic curve from a known point. 
    # //Alex Pilafian 2017 - sikanrong@gmail.com - github.com/sikanrong
    # //If you reuse this code please give me attribution, my dude! I worked hard on this.

    # //parabola defined by ax^2 + bx + c, a and b are passed in to constructor while c is omitted because it isn't relevant to our calculations.
    # //u is known point x-value
    # //L is known length to travel down the curve for our unknown point.
    # //v is the unknown point x-value, once we have v we can calculate the correspondiing unknown y just by pluging it 
    # //back into our parabola function
    import math

    sigFigs = 1000
    # a = 0.1
    # b = 2.0
    # u = 2.0
    # L = 0.2

    du = (b + 2*a*u)
    lu = math.sqrt(1 + math.pow(du, 2))

    def taylor_func():
        return u + (L / lu)

    def newton_func(v):
        dv = (b + 2*a*v)
        lv = math.sqrt(1 + math.pow(dv, 2))
        return v + ((lv*(4*a*L + du*lu - dv*lv + math.asinh(du) - math.asinh(dv)))/(2*a*(2 + 2 * math.pow(dv, 2))))

    #   //get a good first guess for newton from taylor
    firstGuess = taylor_func()

    # //Recursively run newton until it converges on an answer
    def do_approximation(v):
        lastNewton = newton_func(v)
        print(f"<Iterate> {lastNewton}")
        if((v - lastNewton) <= 1.0 / sigFigs):
            return v
        else:
            return do_approximation(lastNewton)

    return do_approximation(firstGuess)


import math


class PathApproximator:
    def __init__(self) -> None:
        self._params = None
        self._poly_degree = 2

    def approximate(self, points: list) -> None:
        xs = [0.0] * len(points)
        ys = [0.0] * len(points)

        for i, (x, y) in enumerate(points):
            xs[i] = x
            ys[i] = y

        self._params = np.polyfit(xs, ys, self._poly_degree)
        self._p = np.poly1d(self._params)

    def calc_y_at_x(self, x):
        return self._p(x)

    def calculate_x_from_distance(self, point: list, L: float) -> list:
        if self._params is None:
            return []

        x, y = point

        sigFigs = 1000
        # # a = 0.1
        # # b = 2.0
        # # u = 2.0
        # # L = 0.2

        # du = (b + 2*a*u)
        # lu = math.sqrt(1 + math.pow(du, 2))

        # def taylor_func():
        #     return u + (L / lu)

        # def newton_func(v):
        #     dv = (b + 2*a*v)
        #     lv = math.sqrt(1 + math.pow(dv, 2))
        #     return v + ((lv*(4*a*L + du*lu - dv*lv + math.asinh(du) - math.asinh(dv)))/(2*a*(2 + 2 * math.pow(dv, 2))))

        #   //get a good first guess for newton from taylor
        first_guess = self._calc_taylor_func(x, L)

        # //Recursively run newton until it converges on an answer
        # def do_approximation(v):
        #     lastNewton = newton_func(v)
        #     print(f"<Iterate> {lastNewton}")
        #     if((v - lastNewton) <= 1.0 / sigFigs):
        #         return v
        #     else:
        #         return do_approximation(lastNewton)

        value = first_guess
        stop_flag = False

        while not stop_flag:
            last_newton = self._calc_newton_func(x, value, L)
            print(f"<Iterate> {last_newton}")
            if((value - last_newton) <= 1.0 / sigFigs):
                value = last_newton
            else:
                stop_flag = True

        return value

    def _calc_dx(self, x):
        a = self._params[0]
        b = self._params[1]

        return b + 2*a*x

    def _calc_taylor_func(self, x, L):
        dx = self._calc_dx(x)
        lu = math.sqrt(1 + math.pow(dx, 2))
        return x + (L / lu)

    def _calc_newton_func(self, x, v, L):
        a = self._params[0]
        b = self._params[1]

        dx = self._calc_dx(x)
        lu = math.sqrt(1 + math.pow(dx, 2))

        dv = (b + 2*a*v)
        lv = math.sqrt(1 + math.pow(dv, 2))
        return v + ((lv*(4*a*L + dx*lu - dv*lv + math.asinh(dx) - math.asinh(dv)))/(2*a*(2 + 2 * math.pow(dv, 2))))


def path_approximation_test():
    points_to_approximate = [
        [0.1, 0.05],
        [1.2, 0.95],
        [2.2, 2.1],
        [2.9, 3.0],
        [4.1, 4.05],
        [5.05, 4.95]
    ]
    approximator = PathApproximator()
    approximator.approximate(points_to_approximate)
    x = 0.5
    y = approximator.calc_y_at_x(x)
    print(f"At x - {x}, y - {y}")
    L = 1.0
    print(f"After {L} meters:")
    x1 = approximator.calculate_x_from_distance([x, y], L)
    y1 = approximator.calc_y_at_x()
    print()


if __name__ == "__main__":
    test_speed_generator()
    # print(test_curve(1.0, 2.0, 3.0, 100))
    # path_approximation_test()
