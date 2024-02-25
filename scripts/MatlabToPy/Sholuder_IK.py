import numpy as np
from scipy.linalg import pinv
from scipy.spatial.transform import Rotation as R

def shoulder_ik(ez):
    tsize = ez.shape[1]
    
    qe = np.array([0.0, 0.0])
    qe_m = np.zeros((2, tsize))
    
    Rz = R.from_euler('XYZ', [0, 0, np.pi]).as_dcm()
    
    for n in range(tsize):
        qe_m[:, n] = qe
        
        if np.abs(ez[2, n]) > np.finfo(float).eps:
            for k in range(5):
                Ry = R.from_euler('XYZ', [0, qe[0], 0]).as_dcm()
                Rx = R.from_euler('XYZ', [qe[1], 0, 0]).as_dcm()
                
                R_matrix = Rz @ Ry @ Rx
                eze = R_matrix[:, 2]
                
                err = ez[:, n] - eze  # 修正点：ezeを3次元から2次元に切り詰める
                
                s1 = np.sin(qe[0])
                c1 = np.cos(qe[0])
                s2 = np.sin(qe[1])
                c2 = np.cos(qe[1])
                
                J = np.array([
                    [-c1*c2, s1*s2],
                    [0, c2],
                    [-s1*c2, -c1*s2]
                ])
                dqe = pinv(J, rcond=1.0e-2) @ err
                qe = qe + 0.5 * dqe[:2]  # 修正点：dqeを3次元から2次元に切り詰める
    
    return qe_m,qe



"""



import numpy as np
from scipy.spatial.transform import Rotation

def shoulder_ik(ez):
    tsize = ez.shape[1]

    qe = np.array([0.0, 0.0])
    qe_m = np.zeros((2, tsize))

    Rz = Rotation.from_euler('XYZ', [0, 0, np.pi]).as_dcm()

    for n in range(tsize):
        qe_m[:, n] = qe

        if np.abs(ez[1, n]) > np.finfo(float).eps:
            for k in range(10):
                Ry = Rotation.from_euler('XYZ', [0, qe[0], 0]).as_dcm()
                Rx = Rotation.from_euler('XYZ', [qe[1], 0, 0]).as_dcm()
                R = Rz @ Ry @ Rx
                eze = R[:, 2]

                err = ez[:, n] - eze

                s1, c1 = np.sin(qe[0]), np.cos(qe[0])
                s2, c2 = np.sin(qe[1]), np.cos(qe[1])

                J = np.array([[-c1 * c2, s1 * s2],
                              [0, c2],
                              [-s1 * c2, -c1 * s2]])

                dqe = np.linalg.pinv(J.T @ J + 0.01 * np.eye(2)) @ J.T @ err
                qe = qe + 0.5 * dqe[:2]  # Use only the first two elements

    return np.degrees(qe_m)  # Convert radians to degrees for better readability



"""
