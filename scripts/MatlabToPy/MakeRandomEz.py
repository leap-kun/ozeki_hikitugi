import numpy as np
from scipy.interpolate import CubicSpline
from scipy.spatial.transform import Rotation as R

def make_random_ez():
    # random q1, q2
    q_min = np.array([-np.pi, -np.pi])
    q_max = np.array([np.pi/2, 0])

    N = 10  # number of Way Points
    Pq = q_min[:, np.newaxis] + np.multiply(np.random.rand(2, N-1), (q_max - q_min)[:, np.newaxis])
    Pq0 = np.array([0, 0])
    Pq = np.column_stack([Pq0, Pq])
    Tq = np.arange(N)  # Assuming MATLAB-style indexing

    Dtime = 0.05
    time = np.arange(0, Tq[-1] + Dtime, Dtime)

    q = cubic_poly_traj(Pq, Tq, time)

    if q is None:
        return None, None, None

    tsize = len(time)
    ez = np.zeros((3, tsize))

    # shoulder kinematics
    Rz = R.from_euler('XYZ', [0, 0, np.pi])

    for n in range(tsize):
        q1 = q[0, n]
        q2 = q[1, n]
        Ry = R.from_euler('XYZ', [0, q1, 0])
        Rx = R.from_euler('XYZ', [q2, 0, 0])
        R_temp = Rz * Ry * Rx
        ez[:, n] = R_temp.apply(np.array([0, 0, 1]))

    return time, q, ez

def cubic_poly_traj(Pq, Tq, time):

    cs1 = CubicSpline(Tq, Pq[0, :])
    cs2 = CubicSpline(Tq, Pq[1, :])

    q1 = cs1(time)
    q2 = cs2(time)

    q = np.vstack((q1, q2))

    return q

# Example usage:
time, q, ez = make_random_ez()



"""

import numpy as np
from scipy.interpolate import CubicSpline
from scipy.spatial.transform import Rotation

def make_random_ez():
    # ------- random q1, q2 -----------
    q_min = np.array([-np.pi, -np.pi])  # q1min q2min
    q_max = np.array([np.pi/2, 0])       # q1max q2max

    N = 10  # Way Pointsの数
    Pq = q_min[:, np.newaxis] + (q_max[:, np.newaxis] - q_min[:, np.newaxis]) * np.random.rand(2, N - 1)
    Pq0 = np.array([0, 0])
    Pq = np.column_stack((Pq0, Pq))
    Tq = np.arange(N)

    Dtime = 0.05
    time = np.arange(0, Tq[-1] + Dtime, Dtime)

    q = cubic_poly_traj(Pq, Tq, time)

    # ------- shoulder kinematics -------
    Rz = Rotation.from_euler('XYZ', [0, 0, np.pi])

    tsize = len(time)
    ez = np.zeros((3, tsize))  # Fix: Initialize ez with shape (3, tsize)

    for n in range(tsize):
        q1, q2 = q[0, n], q[1, n]
        Ry = Rotation.from_euler('XYZ', [0, q1, 0])
        Rx = Rotation.from_euler('XYZ', [q2, 0, 0])
        R = Rz * Ry * Rx
        ez[:, n] = R.as_dcm()[:, 0]  # Fix: Use as_dcm() to obtain the rotation matrix

    return time, q, ez

def cubic_poly_traj(Pq, Tq, time):
    cs1 = CubicSpline(Tq, Pq[0, :])
    cs2 = CubicSpline(Tq, Pq[1, :])

    q1 = cs1(time)
    q2 = cs2(time)

    q = np.vstack((q1, q2))

    return q

# 使い方の例
time, q, ez = make_random_ez()

print(ez)
"""