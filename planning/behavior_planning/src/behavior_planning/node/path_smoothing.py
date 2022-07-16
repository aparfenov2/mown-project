import casadi
import rospy
import numpy as np
from task_behavior_engine.tree import Node, NodeStatus

from geometry_msgs.msg import Point
from enginx_msgs.msg import Route, PointWithSpeed, SpeedGeneratorDebug


class PathSmoothingNode(Node):
    def __init__(self, name, frame, *args, **kwargs):
        super(PathSmoothingNode, self).__init__(name=name,
                                                run_cb=self.run,
                                                *args, **kwargs)
        self._frame = frame

    def run(self, nodedata):
        pass


class PathSmoother:
    def __init__(self) -> None:
        pass

    def run(self, path):
        opti = casadi.Opti()

        W0 = 1.0
        W1 = 1.0
        # W2 = 10.0
        # W3 = 1.0

        KNOTS = len(path)
        X = opti.variable(KNOTS)
        Y = opti.variable(KNOTS)
        DX = opti.variable(KNOTS - 1)
        DY = opti.variable(KNOTS - 1)
        # DPHI = opti.variable(KNOTS - 1)

        cost = 0.0

        opti.subject_to(X[0] == path[0][0])
        opti.subject_to(Y[0] == path[0][1])

        for i in range(1, KNOTS - 1):
            opti.set_initial(X[i], path[i][0])
            opti.set_initial(Y[i], path[i][1])
            opti.subject_to(X[i] == X[i - 1] + DX[i - 1])
            opti.subject_to(Y[i] == Y[i - 1] + DY[i - 1])
            # opti.subject_to(DPHI[i - 1] == casadi.atan2(DY[i], DX[i]) - casadi.atan2(DY[i - 1], DX[i - 1]))

            cost0 = W0 * ((X[i] - path[i][0]) ** 2 + (Y[i] - path[i][1]) ** 2)
            cost1 = W1 * ((DX[i] - DX[i - 1]) ** 2 + (DY[i] - DY[i - 1]) ** 2)

            cost += cost0 + cost1

        last_index = len(path) - 1
        delta_last_index = KNOTS - 2

        opti.subject_to(X[last_index] == path[-1][0])
        opti.subject_to(Y[last_index] == path[-1][1])
        opti.subject_to(X[last_index] == X[last_index - 1] + DX[delta_last_index])
        opti.subject_to(Y[last_index] == Y[last_index - 1] + DY[delta_last_index])
        # cost += W2 * ((DPHI[last_index - 1]) / (casadi.sqrt(DX[last_index - 1] ** 2 + DY[last_index - 1] ** 2)) - K_MAX) ** 2

        opti.minimize(cost)
        # opts = {'ipopt.print_level': 0, 'print_time': 0, 'ipopt.sb': 'yes'}
        opti.solver('ipopt')

        # times = [0.0] * KNOTS

        sol = opti.solve()

        smoothed_x = [0.0] * KNOTS
        smoothed_y = [0.0] * KNOTS

        for i in range(KNOTS):
            smoothed_x[i] = sol.value(X[i])
            smoothed_y[i] = sol.value(Y[i])
        
        return smoothed_x, smoothed_y


class PathSmootherCasadi:
    def __init__(self, w) -> None:
        self._w1 = w[0]
        self._w2 = w[1]

    def smooth(self, points):
        opti = casadi.Opti()

        knots = len(points)

        X = opti.variable(knots)
        Y = opti.variable(knots)

        cost = 0.0

        opti.subject_to(X[0] == points[0][0])
        opti.subject_to(Y[0] == points[0][1])

        for i in range(1, knots - 1):

            cost_1 = self._w1 * ((X[i] - points[i][0]) ** 2 + (Y[i] - points[i][1]) ** 2)
            cost_2 = self._w2 * ((X[i] - (X[i-1] + X[i+1]) / 2.0) ** 2 + (Y[i] - (Y[i-1] + Y[i+1]) / 2.0) ** 2)

            cost += cost_1 + cost_2

        opti.subject_to(X[-1] == points[-1][0])
        opti.subject_to(Y[-1] == points[-1][1])

        opti.minimize(cost)
        # opts = {
        #     'ipopt.print_level': 0,
        #     'print_time': 0,
        #     'ipopt.fixed_variable_treatment': "make_parameter",
        #     # 'ipopt.s_max': 10,
        #     'ipopt.sb': 'yes'
        # }
        opts = {'ipopt.print_level': 0, 'print_time': 0, 'ipopt.sb': 'yes'}
        opti.solver('ipopt', opts)

        debug_x = [0.0] * knots
        debug_y = [0.0] * knots

        try:
            sol = opti.solve()

            for i in range(knots):
                debug_x[i] = sol.value(X[i])
                debug_y[i] = sol.value(Y[i])
        except Exception as e:
            print(e)
            print("HELP")
            for i in range(self._knots):
                debug_x[i] = opti.debug.value(X[i])
                debug_y[i] = opti.debug.value(Y[i])

        return debug_x, debug_y


def test_path_smoothier():
    points = [
        [0.0, 0.0],
        [0.5, 0.0],
        [1.0, 0.0],
        [1.5, 0.0],
        [2.0, 0.0],
        [2.5, 0.0],
        [3.0, 0.0],
        [3.5, 0.0],
        [4.0, 0.0],
        [4.5, 0.0],
        [5.0, 0.0],
        [5.0, 0.5],
        [5.0, 1.0],
        [5.0, 1.5],
        [5.0, 2.0],
        [5.0, 2.5],
        [5.0, 3.0],
        [5.0, 3.5],
        [5.0, 4.0],
        [5.0, 4.5],
    ]
    smoother = PathSmootherCasadi([1.0, 10.0])
    xs, ys = smoother.smooth(points)

    import matplotlib.pyplot as plt
    fig = plt.figure()
    ax = plt.axes()
    _xs, _ys = [], []
    for x, y in points:
        _xs.append(x)
        _ys.append(y)

    ax.plot(xs, ys, label='x')
    ax.plot(_xs, _ys, label='dx')

    plt.legend()
    plt.show()


def spline_interpolation():
    import numpy as np
    from scipy import interpolate

    import matplotlib.pyplot as plt

    #x = np.arange(0, 2*np.pi+np.pi/4, 2*np.pi/8)
    #y = np.sin(x)

    points = [
        [0.0, 0.0],
        [1.0, 0.0],
        [2.0, 0.0],
        [3.0, 0.0],
        [4.0, 0.0],
        [5.0, 0.0],
        [5.0, 1.0],
        [5.0, 2.0],
        [5.0, 3.0],
        [5.0, 4.0],
    ]
    ctr = np.array(points)

    # ctr =np.array( [(3 , 1), (2.5, 4), (0, 1), (-2.5, 4),
    #                 (-3, 0), (-2.5, -4), (0, -1), (2.5, -4), (3, -1)])

    x=ctr[:,0]
    y=ctr[:,1]

    #x=np.append(x,x[0])
    #y=np.append(y,y[0])

    tck,u = interpolate.splprep([x,y],k=3,s=0)
    u=np.linspace(0,1,num=50,endpoint=True)
    out = interpolate.splev(u,tck)

    plt.figure()
    plt.plot(x, y, 'ro', out[0], out[1], 'b')
    plt.legend(['Points', 'Interpolated B-spline', 'True'],loc='best')
    plt.axis([min(x)-1, max(x)+1, min(y)-1, max(y)+1])
    plt.title('B-Spline interpolation')
    plt.show()


def spline_evaluation():
    import numpy as np
    from scipy import interpolate

    import matplotlib.pyplot as plt

    points = [
        [0.0, 0.0],
        [1.0, 0.0],
        [2.0, 0.0],
        [3.0, 0.0],
        [4.0, 0.0],
        [5.0, 0.0],
        [5.0, 1.0],
        [5.0, 2.0],
        [5.0, 3.0],
        [5.0, 4.0],
    ]
    ctr = np.array(points)

    # ctr =np.array( [(3 , 1), (2.5, 4), (0, 1), (-2.5, 4),
    #                 (-3, 0), (-2.5, -4), (0, -1), (2.5, -4), (3, -1),])
    x=ctr[:,0]
    y=ctr[:,1]

    # uncomment both lines for a closed curve
    #x=np.append(x,[x[0]])  
    #y=np.append(y,[y[0]])

    l=len(x)  

    t=np.linspace(0,1,l-2,endpoint=True)
    t=np.append([0,0,0],t)
    t=np.append(t,[1,1,1])

    tck=[t,[x,y],3]
    u3=np.linspace(0,1,(max(l*2,70)),endpoint=True)
    out = interpolate.splev(u3,tck)

    print(u3)
    print()
    print(out)

    plt.plot(x,y,'k--',label='Control polygon',marker='o',markerfacecolor='red')
    #plt.plot(x,y,'ro',label='Control points only')
    plt.plot(out[0],out[1],'b',linewidth=2.0,label='B-spline curve')
    plt.legend(loc='best')
    plt.axis([min(x)-1, max(x)+1, min(y)-1, max(y)+1])
    plt.title('Cubic B-spline curve evaluation')
    plt.show()


if __name__ == "__main__":
    test_path_smoothier()
