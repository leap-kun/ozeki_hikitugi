import numpy as np

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
