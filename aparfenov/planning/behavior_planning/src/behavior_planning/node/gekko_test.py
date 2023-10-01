from cgi import test
import gekko

def test_1():
    #Initialize Model
    m = gekko.GEKKO()

    #define parameter
    eq = m.Param(value=40)

    #initialize variables
    x1, x2, x3, x4 = [m.Var(lb=1, ub=5) for i in range(4)]

    #initial values
    x1.value = 1
    x2.value = 5
    x3.value = 5
    x4.value = 1

    #Equations
    m.Equation(x1*x2*x3*x4>=25)
    m.Equation(x1**2+x2**2+x3**2+x4**2==eq)

    #Objective
    m.Minimize(x1*x4*(x1+x2+x3)+x3)

    #Set global options
    m.options.IMODE = 3 #steady state optimization

    #Solve simulation
    m.solve()

    #Results
    print('')
    print('Results')
    print('x1: ' + str(x1.value))
    print('x2: ' + str(x2.value))
    print('x3: ' + str(x3.value))
    print('x4: ' + str(x4.value))


def test_2():
    m = gekko.GEKKO()

    S = []
    DS = []
    DDS = []
    TIMES = [0.0]

    s = m.Var(1.0, lb=0.0, ub=1.0)
    ds = m.Var(2.0, lb=0.0, ub=2.0)
    dds = m.Var(1.0, lb=-1.0, ub=1.0)

    m.Equation(s == 0.0)
    m.Equation(ds == 0.0)
    m.Equation(dds == 0.0)

    prev_s = s
    prev_ds = ds
    prev_dds = dds

    S.append(prev_s)
    DS.append(prev_ds)
    DDS.append(prev_dds)

    for i in range(1, 40):
        s_i = m.Var(1.0, lb=0.0, ub=1.0)
        ds_i = m.Var(2.0, lb=0.0, ub=2.0)
        dds_i = m.Var(1.0, lb=-1.0, ub=1.0)

        m.Equation(ds_i == prev_ds + dds_i * 0.1)
        m.Equation(s_i == prev_s + prev_ds * 0.1 + dds_i * 0.5 * (0.1 ** 2))

        m.Minimize(30 * (s_i - 1.0) ** 2)
        m.Minimize(100 * (ds_i - 0.5) ** 2)
        m.Minimize(0.1*(dds_i - prev_dds) ** 2)

        prev_s = s_i
        prev_ds = ds_i
        prev_dds = dds_i

        S.append(prev_s)
        DS.append(prev_ds)
        DDS.append(prev_dds)
        TIMES.append(i * 0.1)

    #Set global options
    m.options.IMODE = 3 #steady state optimization

    #Solve simulation
    m.solve()

    print('')
    print('Results')
    print('x1: ' + str(s_i.value))
    print('x2: ' + str(ds_i.value))
    print('x3: ' + str(dds_i.value))

    import matplotlib.pyplot as plt
    fig = plt.figure()
    ax = plt.axes()

    ax.plot(TIMES, S, label='x')
    ax.plot(TIMES, DS, label='dx')
    ax.plot(TIMES, DDS, label='ddx')

    plt.legend()
    plt.show()


test_2()
