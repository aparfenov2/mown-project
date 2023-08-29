#!/usr/bin/env python3
from time import sleep

import casadi


class TrajectorySmoother:
    def __init__(self, w: list, raise_exception=True) -> None:
        self._w1 = w[0]
        self._w2 = w[1]
        self._w3 = w[2]
        self._w4 = w[3]

        self._raise_exception = raise_exception

    def smooth(self, path, robot_pose):
        opti = casadi.Opti()

        W0 = 0.1
        W1 = 100.0
        W2 = 10.0
        # W3 = 1.0

        EPS = 10e-6
        K_MAX = 2.0

        KNOTS = len(path)
        X = opti.variable(KNOTS)
        Y = opti.variable(KNOTS)
        DX = opti.variable(KNOTS - 1)
        DY = opti.variable(KNOTS - 1)
        # DPHI = opti.variable(KNOTS - 1)
        # THETA = opti.variable()

        cost = 0.0

        opti.subject_to(X[0] == robot_pose[0])
        opti.subject_to(Y[0] == robot_pose[1])
        # opti.subject_to(THETA == robot_pose[2])
        # opti.subject_to(THETA == casadi.atan2(DY[0], DX[0] + 10e-6))

        for i in range(1, KNOTS - 1):
            opti.set_initial(X[i], path[i][0])
            opti.set_initial(Y[i], path[i][1])
            opti.subject_to(X[i] == X[i - 1] + DX[i - 1])
            opti.subject_to(Y[i] == Y[i - 1] + DY[i - 1])
            # opti.subject_to(DPHI[i - 1] == casadi.atan2(DY[i], DX[i] + EPS) - casadi.atan2(DY[i - 1], DX[i - 1] + EPS))

            cost0 = W0 * ((X[i] - path[i][0]) ** 2 + (Y[i] - path[i][1]) ** 2)
            # cost1 = W1 * ((DX[i] - DX[i - 1]) ** 2 + (DY[i] - DY[i - 1]) ** 2)
            # cost2 = W2 * (DPHI[i] - DPHI[i - 1]) ** 2
            cost3 = W1 * ((X[i] - (X[i-1] + X[i+1]) / 2.0) ** 2 + (Y[i] - (Y[i-1] + Y[i+1]) / 2.0) ** 2)
            # cost2 = W2 * ((DPHI[i - 1]) / (casadi.sqrt(DX[i - 1] ** 2 + DY[i - 1] ** 2) + EPS) - K_MAX) ** 2

            # cost += cost0 + cost1 + cost2
            cost += cost3 + cost0

        last_index = len(path) - 1
        delta_last_index = KNOTS - 2

        opti.subject_to(X[last_index] == path[-1][0])
        opti.subject_to(Y[last_index] == path[-1][1])
        opti.subject_to(X[last_index] == X[last_index - 1] + DX[delta_last_index])
        opti.subject_to(Y[last_index] == Y[last_index - 1] + DY[delta_last_index])
        # cost += W2 * ((DPHI[last_index - 1]) / (casadi.sqrt(DX[last_index - 1] ** 2 + DY[last_index - 1] ** 2)) - K_MAX) ** 2

        opti.minimize(cost)
        opts = {'ipopt.print_level': 0, 'print_time': 0, 'ipopt.sb': 'yes'}
        opti.solver('ipopt', opts)

        # times = [0.0] * KNOTS

        try:
            sol = opti.solve()

            trajectory = {
                'x': [0.0] * KNOTS,
                'y': [0.0] * KNOTS
            }

            for i in range(KNOTS):
                trajectory['x'][i] = sol.value(X[i])
                trajectory['y'][i] = sol.value(Y[i])

            return trajectory
        except Exception as e:
            if self._raise_exception:
                save_data = {
                    "robot_position": robot_pose,
                    "path": path
                }

                import json
                with open('/tmp/json_data.json', 'w') as outfile:
                    json.dump(save_data, outfile)

                raise Exception()
            else: 
                trajectory = {
                    'x': [0.0] * KNOTS,
                    'y': [0.0] * KNOTS
                }

                for i in range(KNOTS):
                    trajectory['x'][i] = opti.debug.value(X[i])
                    trajectory['y'][i] = opti.debug.value(Y[i])
                
                return trajectory
