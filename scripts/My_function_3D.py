import numpy as np
from scipy.linalg import pinv
from scipy.spatial.transform import Rotation as R
from pykalman import KalmanFilter 
import math
from scipy.ndimage import convolve
from scipy.signal import butter, sosfreqz, sosfilt
from scipy.ndimage import median_filter


"""
Make My function
"""

"""
 [ INFO] [1698212587.545685610]: 	 Depth:
 [ INFO] [1698212587.545695464]: 		 Extrinsics:
 [ INFO] [1698212587.545721572]: 			 Translation: 0, 0, 0
 [ INFO] [1698212587.545738339]: 			 Rotation[0]: 1, 0, 0
 [ INFO] [1698212587.545754204]: 			 Rotation[1]: 0, 1, 0
 [ INFO] [1698212587.545765890]: 			 Rotation[2]: 0, 0, 1
"""

#カメラ画像からロボット座標に変換

"""
def camera_to_robot_coordinates(point):
    
    rotation = np.array([[1, 0, 0],
                         [0, 1, 0],
                         [0, 0, 1]])
    
    translation = np.array([[0, 0, 0]])
    
    camera_coordinates = np.array([[point.x,point.y,point.z]])
    
    temp = camera_coordinates.T - translation.T
    
    robot_coordinate = np.dot(rotation.T,temp)
    
    #横軸
    robot_y = (robot_coordinate[0])
    #縦軸
    robot_z = -(robot_coordinate[1])
    #奥行き
    robot_x = (robot_coordinate[2])
    
    return robot_y, robot_z, robot_x
"""

def camera_to_robot_coordinates(point):
    
    rotation = np.array([[1, 0, 0],
                         [0, 1, 0],
                         [0, 0, 1]])
    
    translation = np.array([[0, 0, 0]])
    
    camera_coordinates = np.array([[point.x,point.y,point.z]])
    
    temp = camera_coordinates.T - translation.T
    
    robot_coordinate = np.dot(rotation.T,temp)
    
    #横軸
    robot_y = -(robot_coordinate[0])
    #縦軸
    robot_z = -(robot_coordinate[1])
    #奥行き
    robot_x = (robot_coordinate[2])
    
    robot_xyz = np.array([robot_x,robot_y,robot_z])
    
    return robot_xyz

#degの角度範囲は-15°~225°
#y0,z0は始点,y1,z1は始点の番号1つ前,y2,z2は始点の番号1つ後


def atan2_deg(y0, y1, y2, z0, z1, z2):
    
    #y0,z0 = 右肩, y1,z1 = 首,y2,z2 = 右肘
    
    # ベクトル1とベクトル2のなす角を計算します
    angle1 = np.arctan2(z1 - z0, y1 - y0)
    
    # ベクトル2とベクトル3のなす角を計算します
    angle2 = np.arctan2(z2 - z0, y2 - y0)
    
    # 三点のなす角を計算します
    total_angle = angle1 - angle2
    
    #値域:-pi~piから0~2*piに変更する部分
    
    if total_angle < 0:
        total_angle += 2 * np.pi
    
    #ラジアンからdegreeに変換
    degree = np.rad2deg(total_angle)
    
    
    #モーター可動域範囲の上限値設定
    if(degree > 225):
        degree = 225
    elif(degree < -45):
        degree = -45

    return degree


#余弦定理
def cosine_xyz(x2, x1, x3, y2, y1, y3, z2, z1, z3, rot):
    # ベクトル化して計算
    A = np.array([x2 - x1, y2 - y1, z2 - z1])
    B = np.array([x3 - x2, y3 - y2, z3 - z2])
    
    # 三平方の定理を使用
    A_squared = np.sum(A**2)
    B_squared = np.sum(B**2)
    
    # 始点と終点の長さ
    C_squared = np.sum((np.array([x3 - x1, y3 - y1, z3 - z1]))**2)
    
    # 余弦定理を用いて角度を計算
    cos = (C_squared - A_squared - B_squared) / (2.0 * np.sqrt(A_squared * B_squared))
    
    # 角度を計算
    theta = np.arccos(np.clip(cos, -1.0, 1.0))
    
    deg = rot * theta * 180 / np.pi

    return deg


#角度の制限
def limit_deg(input_deg,max_deg,min_deg):
    
    if input_deg < min_deg:
        input_deg = min_deg
    elif input_deg > max_deg:
        input_deg = max_deg
    
    return input_deg


#下記の関数は,NANが出た場合に一つ前の値を参照にして値を書き換える関数
def replace_nan_with_previous(lst):
    result = np.array([])
    previous_value = 0

    for current_value in lst:
        if not np.isnan(current_value):
            previous_value = current_value
        result = np.append(result, previous_value)

    return result


def replace_nan_with_previous_ez(ez_array):
    result = np.zeros_like(ez_array)

    previous_values = np.zeros(3)  # ez_arrayが3つの要素(x, y, z)で構成されていると仮定しています

    for i in range(0, len(ez_array), 3):
        current_ez = ez_array[i:i+3]

        for j in range(3):
            current_value = current_ez[j]

            if not np.isnan(current_value):
                previous_values[j] = current_value

            result[i+j] = previous_values[j]
    
    last_values = result[-3:]
    result_matrix = np.array(last_values).reshape((3, 1))

    return result_matrix

#下記の関数は肩から腕の単位ベクトルを摘出する。
def ChUnitVector(p_Sholuder,p_Elbow):

    r = p_Sholuder - p_Elbow
    norm_r = np.linalg.norm(r)
    
    e = r /norm_r
    
    return e


def get_sholuder_IK(p_Sholuder,p_Elbow,q_deg_org):
    
    
    #r = np.array([point_1[0] - point_2[0], point_1[1] - point_2[1], point_1[2] - point_2[2]])
    r = p_Sholuder - p_Elbow
    norm_r = np.linalg.norm(r)
    
    e = r /norm_r
    
    e_x, e_y, e_z = e

    q_org = np.deg2rad(q_deg_org)

    if e_z > 0.5:
        q2 = np.arcsin(e_y)
    else:        
        if np.abs(np.sin(q_org[0])) > np.abs(np.cos(q_org[0])):
            _c2 = -e_x/np.sin(q_org[0])
        else:
            _c2 = e_z/np.cos(q_org[0])
        q2 = np.arctan2(e_y, _c2)
    
    c2 = np.cos(q2)

    
    if np.abs(c2) > np.deg2rad(25): #np.finfo(float).eps
        q1 = np.arctan2( -e_x / c2, e_z / c2)
    else:
        q1 = q_org[0]
    
    
    
    theta1 = np.rad2deg(q1)
    theta2 = np.rad2deg(q2)
    
    """
    theta2_a = np.degrees(q2a)
    theta2_b = np.degrees(q2b)
    """

    
    return r,theta1,theta2


def shoulder_ik_np(ez):
    
    tsize = ez.shape[1]
    
    qe = np.array([0.0, 0.0])
    qe_m = np.zeros((2, tsize))
    
    Rz = np.array([
        [np.cos(np.pi), -np.sin(np.pi), 0],
        [np.sin(np.pi), np.cos(np.pi), 0],
        [0, 0, 1]
    ])
    
    for n in range(tsize):
        qe_m[:, n] = qe
        
        if np.abs(ez[2, n]) > np.finfo(float).eps:
            for k in range(5):
                Ry = np.array([
                    [np.cos(0), 0, np.sin(0)],
                    [0, 1, 0],
                    [-np.sin(0), 0, np.cos(0)]
                ])
                
                Rx = np.array([
                    [1, 0, 0],
                    [0, np.cos(qe[1]), -np.sin(qe[1])],
                    [0, np.sin(qe[1]), np.cos(qe[1])]
                ])
                
                R_matrix = np.dot(Rz, np.dot(Ry, Rx))
                eze = R_matrix[:, 2]
                
                err = ez[:, n] - eze
                
                s1, c1 = np.sin(qe[0]), np.cos(qe[0])
                s2, c2 = np.sin(qe[1]), np.cos(qe[1])
                
                J = np.array([
                    [-c1 * c2, s1 * s2],
                    [0, c2],
                    [-s1 * c2, -c1 * s2]
                ])
                dqe = np.linalg.pinv(J, rcond=1.0e-2) @ err
                qe = qe + 0.5 * dqe[:2]
            
                
                theta1 = np.rad2deg(qe[0])
                theta2 = np.rad2deg(qe[1])
    
    
    return theta1,theta2


def shoulder_ik(ez):
    tsize = ez.shape[1]
    
    qe = np.array([0.0, 0.0])
    
    # qeの最小値,最大値
    qe[0] = np.clip(qe[0], -np.pi, np.pi/2)
    qe[1] = np.clip(qe[1], -np.pi, 0)
    
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
                

    return qe[0],qe[1]


def camera_to_robot_coordinates_xyz(point):
    
    rotation = np.array([[1, 0, 0],
                         [0, 1, 0],
                         [0, 0, 1]])
    
    translation = np.array([[0, 0, 0]])
    
    camera_coordinates = np.array([[point.x,point.y,point.z]])
    
    temp = camera_coordinates.T - translation.T
    
    robot_coordinate = np.dot(rotation.T,temp)
    
    #横軸
    robot_y = -(robot_coordinate[0])
    #縦軸
    robot_z = -(robot_coordinate[1])
    #奥行き
    robot_x = (robot_coordinate[2])
    
    return robot_x,robot_y,robot_z
