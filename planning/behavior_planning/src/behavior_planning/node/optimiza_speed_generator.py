import pickle
import time
import numpy as np
from hyperopt import fmin, tpe, hp, STATUS_OK

from behavior_planning.node.speed_generator_node import NonlinaerSmootherCasadi


            # [10.0, 0.5, 0.2], [10.0, 0.5, 0.0],
            # [5.0, 0.5, 0.2], [5.0, 0.8, 0.0],
            # [1.0, 0.8, 0.2], [1.0, 0.8, 0.0]

class MPCOptimizer:

    def objective(self, args):
        w1 = args['w1']
        w2 = args['w2']
        w3 = 1.0
        w4 = 1.0
        controller = NonlinaerSmootherCasadi(21, 0.1, [w1, w2, w3, w4])

        cost = 0.0
        for s, ds, start_ds in [
            [1.0, 0.5, 0.2], [1.0, 0.5, 0.0],
            [1.0, 0.7, 0.2], [1.0, 0.7, 0.0],
            [1.0, 0.8, 0.2], [1.0, 0.8, 0.0]
        ]:
            try:
                xs, dxs, ddxs, times = controller.smooth(
                    [0.0, start_ds, 0.0],
                    [0.0, s],
                    [0.0, 1.5], [-5.0, 5.0], [-5.0, 5.0],
                    [s, ds]
                )
            except:
                return np.inf


            for x, dx in zip(xs, dxs):
                # cost += (x - s) ** 2
                cost += 10 * (dx - ds) ** 2
                # if dx > ds:
                #     cost += 100.0
  
        return cost


def optimize_mpc():
    space = {
        'w1': hp.uniform('w1', 0.0001, 10.0),
        'w2': hp.uniform('w2', 0.0001, 10.0),
        # 'w3': hp.uniform('w3', 0.001, 10000),
        # 'w4': hp.uniform('w4', 0.1, 10000)
        # 'w1': hp.choice('w1', [0.1, 1.0, 10.0, 100.0, 1000.0, 10000]),
        # 'w2': hp.choice('w2', [0.1, 1.0, 10.0, 100.0, 1000.0, 10000]),
        # 'w3': hp.choice('w3', [0.1, 1.0, 10.0, 100.0, 1000.0, 10000]),
        # 'w4': hp.choice('w4', [0.1, 1.0, 10.0, 100.0, 1000.0, 10000])
    }

    optimizer = MPCOptimizer()

    best = fmin(optimizer.objective,
        space=space,
        algo=tpe.suggest,
        max_evals=100
    )

    print(best)

optimize_mpc()
