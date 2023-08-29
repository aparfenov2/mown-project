import pickle
import time
import numpy as np
from hyperopt import fmin, tpe, hp, STATUS_OK

from move_controller.move_nmpc import NonLinearMPC, StopNonlinearMPC


class MPCOptimizer:
    def __init__(self) -> None:
        
        self.target = [
            [0.1, 0.0, 1.0],
            [0.2, 0.0, 1.0],
            [0.3, 0.0, 0.0],
            [0.3, 0.0, 0.0],
            [0.3, 0.0, 0.0],
            [0.3, 0.0, 0.0],
            [0.3, 0.0, 0.0],
            [0.3, 0.0, 0.0],
            [0.3, 0.0, 0.0],
            [0.3, 0.0, 0.0],
            [0.3, 0.0, 0.0],
            [0.3, 0.0, 0.0],
            [0.3, 0.0, 0.0],
            [0.3, 0.0, 0.0],
        ]

    def objective(self, args):
        w1 = args['w1']
        w2 = args['w2']
        w3 = args['w3']
        controller = NonLinearMPC(15, 0.1, [w1, w2, w3])
        debug, times = controller.calculate(
            [0.0, 0.0, 0.0, 0.0],
            [0.0, 3.0],
            [-5.0, 5.0],
            [-5.0, 5.0],
            [-5.0, 5.0],
            self.target
        )

        cost = 0.0

        for x, y, v, target in zip(debug['x'], debug['y'], debug['v'], self.target):
            # cost += (target[0] - x) ** 2
            # cost += (target[1] - y) ** 2
            cost += (target[2] - v) ** 2
        
        return cost


class StopMPCOptimizer:
    def __init__(self) -> None:
        pass

    def objective(self, args):
        w1 = args['w1']
        w2 = args['w2']
        w3 = args['w3']
        w4 = args['w4']
        controller = StopNonlinearMPC(15, 0.1, [w1, w2, w3, w4])
        debug, times = controller.calculate(
            [0.0, 0.0, 0.0, 0.0],
            [0.0, 3.0],
            [-5.0, 5.0],
            [-5.0, 5.0],
            [-5.0, 5.0]
        )

        cost = 0.0

        for v, w in zip(debug['v'], debug['w']):
            # cost += (target[0] - x) ** 2
            # cost += (target[1] - y) ** 2
            cost += (v) ** 2
            cost += (w) ** 2
        
        return cost

# def objective(x, y):
#     return x ** 2

def objective(params):
    x, y = params['x'], params['y']
    return np.sin(np.sqrt(x**2 + y**2))

def optimize_mpc():
    space = {
        'w1': hp.uniform('w1', 0.1, 10000),
        'w2': hp.uniform('w2', 0.1, 10000),
        'w3': hp.uniform('w3', 0.1, 10000)
    }

    optimizer = MPCOptimizer()

    best = fmin(optimizer.objective,
        space=space,
        algo=tpe.suggest,
        max_evals=100
    )

    print(best)


def optimize_stop():
    space = {
        'w1': hp.uniform('w1', 0.01, 10000),
        'w2': hp.uniform('w2', 0.01, 10000),
        'w3': hp.uniform('w3', 0.01, 10000),
        'w4': hp.uniform('w4', 0.01, 10000)
    }

    optimizer = StopMPCOptimizer()

    best = fmin(optimizer.objective,
        space=space,
        algo=tpe.suggest,
        max_evals=100
    )

    print(best)

optimize_mpc()