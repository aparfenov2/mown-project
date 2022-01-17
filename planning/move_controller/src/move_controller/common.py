import numpy as np


def convert_orientation(theta):
    return np.math.atan2(np.sin(theta), np.cos(theta))


def angles_difference(a1, a2):
    # a = a2 - a1

    # if a > np.pi:
    #     a -= 2*np.pi
    # elif a < -np.pi:
    #     a += 2*np.pi
    
    return convert_orientation(a2-a1)


def compute_theta(p1, p2):
    return np.math.atan2(p2[1] - p1[1], p2[0] - p1[0])