# -*- coding: utf-8 -*-
# pylint: disable=invalid-name, too-many-arguments, too-many-branches,
# pylint: disable=too-many-locals, too-many-instance-attributes, too-many-lines

"""
This module implements the linear Kalman filter in both an object
oriented and procedural form. The KalmanFilter class implements
the filter by storing the various matrices in instance variables,
minimizing the amount of bookkeeping you have to do.

All Kalman filters operate with a predict->update cycle. The
predict step, implemented with the method or function predict(),
uses the state transition matrix F to predict the state in the next
time period (epoch). The state is stored as a gaussian (x, P), where
x is the state (column) vector, and P is its covariance. Covariance
matrix Q specifies the process covariance. In Bayesian terms, this
prediction is called the *prior*, which you can think of colloquially
as the estimate prior to incorporating the measurement.

The update step, implemented with the method or function `update()`,
incorporates the measurement z with covariance R, into the state
estimate (x, P). The class stores the system uncertainty in S,
the innovation (residual between prediction and measurement in
measurement space) in y, and the Kalman gain in k. The procedural
form returns these variables to you. In Bayesian terms this computes
the *posterior* - the estimate after the information from the
measurement is incorporated.

Whether you use the OO form or procedural form is up to you. If
matrices such as H, R, and F are changing each epoch, you'll probably
opt to use the procedural form. If they are unchanging, the OO
form is perhaps easier to use since you won't need to keep track
of these matrices. This is especially useful if you are implementing
banks of filters or comparing various KF designs for performance;
a trivial coding bug could lead to using the wrong sets of matrices.

This module also offers an implementation of the RTS smoother, and
other helper functions, such as log likelihood computations.

The Saver class allows you to easily save the state of the
KalmanFilter class after every update

This module expects NumPy arrays for all values that expect
arrays, although in a few cases, particularly method parameters,
it will accept types that convert to NumPy arrays, such as lists
of lists. These exceptions are documented in the method or function.

Examples
--------
The following example constructs a constant velocity kinematic
filter, filters noisy data, and plots the results. It also demonstrates
using the Saver class to save the state of the filter at each epoch.

.. code-block:: Python

    import matplotlib.pyplot as plt
    import numpy as np
    from filterpy.kalman import KalmanFilter
    from filterpy.common import Q_discrete_white_noise, Saver

    r_std, q_std = 2., 0.003
    cv = KalmanFilter(dim_x=2, dim_z=1)
    cv.x = np.array([[0., 1.]]) # position, velocity
    cv.F = np.array([[1, dt],[ [0, 1]])
    cv.R = np.array([[r_std^^2]])
    f.H = np.array([[1., 0.]])
    f.P = np.diag([.1^^2, .03^^2)
    f.Q = Q_discrete_white_noise(2, dt, q_std**2)

    saver = Saver(cv)
    for z in range(100):
        cv.predict()
        cv.update([z + randn() * r_std])
        saver.save() # save the filter's state

    saver.to_array()
    plt.plot(saver.x[:, 0])

    # plot all of the priors
    plt.plot(saver.x_prior[:, 0])

    # plot mahalanobis distance
    plt.figure()
    plt.plot(saver.mahalanobis)

This code implements the same filter using the procedural form

    x = np.array([[0., 1.]]) # position, velocity
    F = np.array([[1, dt],[ [0, 1]])
    R = np.array([[r_std^^2]])
    H = np.array([[1., 0.]])
    P = np.diag([.1^^2, .03^^2)
    Q = Q_discrete_white_noise(2, dt, q_std**2)

    for z in range(100):
        x, P = predict(x, P, F=F, Q=Q)
        x, P = update(x, P, z=[z + randn() * r_std], R=R, H=H)
        xs.append(x[0, 0])
    plt.plot(xs)


For more examples see the test subdirectory, or refer to the
book cited below. In it I both teach Kalman filtering from basic
principles, and teach the use of this library in great detail.

FilterPy library.
http://github.com/rlabbe/filterpy

Documentation at:
https://filterpy.readthedocs.org

Supporting book at:
https://github.com/rlabbe/Kalman-and-Bayesian-Filters-in-Python

This is licensed under an MIT license. See the readme.MD file
for more information.

Copyright 2014-2018 Roger R Labbe Jr.
"""

from __future__ import absolute_import, division

from copy import deepcopy
from math import log, exp, sqrt
import sys
import numpy as np
from numpy import dot, zeros, eye, isscalar, shape
import numpy.linalg as linalg


class KalmanFilter(object):
    r"""Implements a Kalman filter. You are responsible for setting the
    various state variables to reasonable values; the defaults  will
    not give you a functional filter.

    For now the best documentation is my free book Kalman and Bayesian
    Filters in Python [2]_. The test files in this directory also give you a
    basic idea of use, albeit without much description.

    In brief, you will first construct this object, specifying the size of
    the state vector with dim_x and the size of the measurement vector that
    you will be using with dim_z. These are mostly used to perform size checks
    when you assign values to the various matrices. For example, if you
    specified dim_z=2 and then try to assign a 3x3 matrix to R (the
    measurement noise matrix you will get an assert exception because R
    should be 2x2. (If for whatever reason you need to alter the size of
    things midstream just use the underscore version of the matrices to
    assign directly: your_filter._R = a_3x3_matrix.)

    After construction the filter will have default matrices created for you,
    but you must specify the values for each. It’s usually easiest to just
    overwrite them rather than assign to each element yourself. This will be
    clearer in the example below. All are of type numpy.array.


    Examples
    --------

    Here is a filter that tracks position and velocity using a sensor that only
    reads position.

    First construct the object with the required dimensionality. Here the state
    (`dim_x`) has 2 coefficients (position and velocity), and the measurement
    (`dim_z`) has one. In FilterPy `x` is the state, `z` is the measurement.

    .. code::

        from filterpy.kalman import KalmanFilter
        f = KalmanFilter (dim_x=2, dim_z=1)


    Assign the initial value for the state (position and velocity). You can do this
    with a two dimensional array like so:

        .. code::

            f.x = np.array([[2.],    # position
                            [0.]])   # velocity

    or just use a one dimensional array, which I prefer doing.

    .. code::

        f.x = np.array([2., 0.])


    Define the state transition matrix:

        .. code::

            f.F = np.array([[1.,1.],
                            [0.,1.]])

    Define the measurement function. Here we need to convert a position-velocity
    vector into just a position vector, so we use:

        .. code::

        f.H = np.array([[1., 0.]])

    Define the state's covariance matrix P.

    .. code::

        f.P = np.array([[1000.,    0.],
                        [   0., 1000.] ])

    Now assign the measurement noise. Here the dimension is 1x1, so I can
    use a scalar

    .. code::

        f.R = 5

    I could have done this instead:

    .. code::

        f.R = np.array([[5.]])

    Note that this must be a 2 dimensional array.

    Finally, I will assign the process noise. Here I will take advantage of
    another FilterPy library function:

    .. code::

        from filterpy.common import Q_discrete_white_noise
        f.Q = Q_discrete_white_noise(dim=2, dt=0.1, var=0.13)


    Now just perform the standard predict/update loop:

    .. code::

        while some_condition_is_true:
            z = get_sensor_reading()
            f.predict()
            f.update(z)

            do_something_with_estimate (f.x)


    **Procedural Form**

    This module also contains stand alone functions to perform Kalman filtering.
    Use these if you are not a fan of objects.

    **Example**

    .. code::

        while True:
            z, R = read_sensor()
            x, P = predict(x, P, F, Q)
            x, P = update(x, P, z, R, H)

    See my book Kalman and Bayesian Filters in Python [2]_.


    You will have to set the following attributes after constructing this
    object for the filter to perform properly. Please note that there are
    various checks in place to ensure that you have made everything the
    'correct' size. However, it is possible to provide incorrectly sized
    arrays such that the linear algebra can not perform an operation.
    It can also fail silently - you can end up with matrices of a size that
    allows the linear algebra to work, but are the wrong shape for the problem
    you are trying to solve.

    Parameters
    ----------
    dim_x : int
        Number of state variables for the Kalman filter. For example, if
        you are tracking the position and velocity of an object in two
        dimensions, dim_x would be 4.
        This is used to set the default size of P, Q, and u

    dim_z : int
        Number of of measurement inputs. For example, if the sensor
        provides you with position in (x,y), dim_z would be 2.

    dim_u : int (optional)
        size of the control input, if it is being used.
        Default value of 0 indicates it is not used.

    compute_log_likelihood : bool (default = True)
        Computes log likelihood by default, but this can be a slow
        computation, so if you never use it you can turn this computation
        off.

    Attributes
    ----------
    x : numpy.array(dim_x, 1)
        Current state estimate. Any call to update() or predict() updates
        this variable.

    P : numpy.array(dim_x, dim_x)
        Current state covariance matrix. Any call to update() or predict()
        updates this variable.

    x_prior : numpy.array(dim_x, 1)
        Prior (predicted) state estimate. The *_prior and *_post attributes
        are for convenience; they store the  prior and posterior of the
        current epoch. Read Only.

    P_prior : numpy.array(dim_x, dim_x)
        Prior (predicted) state covariance matrix. Read Only.

    x_post : numpy.array(dim_x, 1)
        Posterior (updated) state estimate. Read Only.

    P_post : numpy.array(dim_x, dim_x)
        Posterior (updated) state covariance matrix. Read Only.

    z : numpy.array
        Last measurement used in update(). Read only.

    R : numpy.array(dim_z, dim_z)
        Measurement noise covariance matrix. Also known as the
        observation covariance.

    Q : numpy.array(dim_x, dim_x)
        Process noise covariance matrix. Also known as the transition
        covariance.

    F : numpy.array()
        State Transition matrix. Also known as `A` in some formulation.

    H : numpy.array(dim_z, dim_x)
        Measurement function. Also known as the observation matrix, or as `C`.

    y : numpy.array
        Residual of the update step. Read only.

    K : numpy.array(dim_x, dim_z)
        Kalman gain of the update step. Read only.

    S :  numpy.array
        System uncertainty (P projected to measurement space). Read only.

    SI :  numpy.array
        Inverse system uncertainty. Read only.

    log_likelihood : float
        log-likelihood of the last measurement. Read only.

    likelihood : float
        likelihood of last measurement. Read only.

        Computed from the log-likelihood. The log-likelihood can be very
        small,  meaning a large negative value such as -28000. Taking the
        exp() of that results in 0.0, which can break typical algorithms
        which multiply by this value, so by default we always return a
        number >= sys.float_info.min.

    mahalanobis : float
        mahalanobis distance of the innovation. Read only.

    inv : function, default numpy.linalg.inv
        If you prefer another inverse function, such as the Moore-Penrose
        pseudo inverse, set it to that instead: kf.inv = np.linalg.pinv

        This is only used to invert self.S. If you know it is diagonal, you
        might choose to set it to filterpy.common.inv_diagonal, which is
        several times faster than numpy.linalg.inv for diagonal matrices.

    alpha : float
        Fading memory setting. 1.0 gives the normal Kalman filter, and
        values slightly larger than 1.0 (such as 1.02) give a fading
        memory effect - previous measurements have less influence on the
        filter's estimates. This formulation of the Fading memory filter
        (there are many) is due to Dan Simon [1]_.

    References
    ----------

    .. [1] Dan Simon. "Optimal State Estimation." John Wiley & Sons.
       p. 208-212. (2006)

    .. [2] Roger Labbe. "Kalman and Bayesian Filters in Python"
       https://github.com/rlabbe/Kalman-and-Bayesian-Filters-in-Python

    """

    def __init__(self, dim_x, dim_z):
        if dim_x < 1:
            raise ValueError("dim_x must be 1 or greater")
        if dim_z < 1:
            raise ValueError("dim_z must be 1 or greater")

        self.dim_x = dim_x
        self.dim_z = dim_z

        self.x = zeros((dim_x, 1))  # state
        self.P = eye(dim_x)  # uncertainty covariance
        self.Q = eye(dim_x)  # process uncertainty
        self.B = None  # control transition matrix
        self.F = eye(dim_x)  # state transition matrix
        self.H = zeros((dim_z, dim_x))  # measurement function
        self.R = eye(dim_z)  # measurement uncertainty
        self._alpha_sq = 1.0  # fading memory control
        self.M = np.zeros((dim_x, dim_z))  # process-measurement cross correlation
        self.z = np.array([[None] * self.dim_z]).T

        # gain and residual are computed during the innovation step. We
        # save them so that in case you want to inspect them for various
        # purposes
        self.K = np.zeros((dim_x, dim_z))  # kalman gain
        self.y = zeros((dim_z, 1))
        self.S = np.zeros((dim_z, dim_z))  # system uncertainty
        self.SI = np.zeros((dim_z, dim_z))  # inverse system uncertainty

        # identity matrix. Do not alter this.
        self._I = np.eye(dim_x)

        self.inv = np.linalg.inv

    @property
    def state(self):
        return (self.x[0], self.x[1])

    def predict(self):
        # x = Fx + Bu
        self.x = dot(self.F, self.x)

        # P = FPF' + Q
        self.P = self._alpha_sq * dot(dot(self.F, self.P), self.F.T) + self.Q

    def update(self, z):
        # y = z - Hx
        # error (residual) between measurement and prediction
        self.y = z - dot(self.H, self.x)

        # common subexpression for speed
        PHT = dot(self.P, self.H.T)

        # S = HPH' + R
        # project system uncertainty into measurement space
        self.S = dot(self.H, PHT) + self.R
        self.SI = self.inv(self.S)
        # K = PH'inv(S)
        # map system uncertainty into kalman gain
        self.K = dot(PHT, self.SI)

        # x = x + Ky
        # predict new x with residual scaled by the kalman gain
        self.x = self.x + dot(self.K, self.y)

        # P = (I-KH)P(I-KH)' + KRK'
        # This is more numerically stable
        # and works for non-optimal K vs the equation
        # P = (I-KH)P usually seen in the literature.

        I_KH = self._I - dot(self.K, self.H)
        self.P = dot(dot(I_KH, self.P), I_KH.T) + dot(dot(self.K, self.R), self.K.T)
