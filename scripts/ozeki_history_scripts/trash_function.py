
import numpy as np

def get_sholuder_IK(x1,y1,z1,x2,y2,z2,q_org):
    
    
    r = np.array([x1 - x2, y1 - y2, z1 - z2])
    norm_r = np.linalg.norm(r)
    
    e = r /norm_r
    
    e_x, e_y, e_z = e
    
    q2a = np.arcsin(e_y)
    
    if q2a > 0:
        q2b = np.pi - q2a
    else:
        #コメントアウトの方な気がする.........
        #q2b = -np.pi + q2a
        q2b = -np.pi - q2a

    if np.abs(q2a - q_org[1]) < np.abs(q2b - q_org[1]):
        q2 = q2a
    else:
        q2 = q2b

    c2 = np.cos(q2)

    if np.abs(c2) > np.finfo(float).eps:
        q1 = np.arctan2( -e_x / c2, e_z / c2)
    else:
        q1 = q_org[0]
        
    theta1 = np.degrees(q1)
    theta2 = np.degrees(q2)

    return theta1,theta2
      
#以下は使っていない関数


#x2,y2,z2に当たる部分は求めたい部位の座標
def cosine_xz(x2,x1,x3,z2,z1,z3):
    #A,B,Cはそれぞれ,三角形の辺の長さ
    A = np.sqrt((x2 - x1)** 2  + (z2 - z1) ** 2)
    B = np.sqrt((x3 - x2) ** 2  + (z3 - z2) ** 2)
    #始点と終点の長さ
    C = np.sqrt((x3 - x1) ** 2 +  (z3 - z1)**2)
    #余弦定理を用いて作成した角度
    cos = (C**2 - A**2 - B**2) / (2.0 * A * B)
    
    if cos >= 1:
        theta = 0.0
    elif cos <= -1:
        theta = np.pi
    else:
        theta = np.arccos(cos)
    
    deg = theta * 180 / np.pi
    
    return deg



#下記の関数は,appendを用いずにジェネレーターで機能を実現したもの
def replace_nan_with_previous_generator(lst):
    previous_value = None

    for current_value in lst:
        if not np.isnan(current_value):
            yield current_value
            previous_value = current_value
        elif previous_value is not None:
            yield previous_value

#下記の関数はmap関数を用いて作成したもの(NAN埋め)

def replace_nan_with_previous_map(lst):
    result = np.array([])
    previous_value = None
    
    def replace_func(current_value):
        nonlocal previous_value
        if not np.isnan(current_value):
            result.append(current_value)
            previous_value = current_value
        elif previous_value is not None:
            result.append(previous_value)

    list(map(replace_func, lst))

    return result

def cosine_xyz(x2, x1, x3, y2, y1, y3, z2, z1, z3, rotation):
    
    # A, B, Cはそれぞれ三角形の辺の長さ
    A = np.sqrt((x2 - x1)**2 + (y2 - y1)**2 + (z2 - z1)**2)
    B = np.sqrt((x3 - x2)**2 + (y3 - y2)**2 + (z3 - z2)**2)
    # 始点と終点の長さ
    C = np.sqrt((x3 - x1)**2 + (y3 - y1)**2 + (z3 - z1)**2)
    # 余弦定理を用いて作成した角度
    cos = (C**2 - A**2 - B**2) / (2.0 * A * B)

    if cos >= 1:
        theta = 0.0
    elif cos <= -1:
        theta = np.pi
    else:
        theta = np.arccos(cos)

    deg = rotation * theta * 180 / np.pi

    return deg
    

#とりあえず,NANを0埋めした
def cosine_xyz(x2, x1, x3, y2, y1, y3, z2, z1, z3,rotation,temp_deg):
    

    # A, B, Cはそれぞれ三角形の辺の長さ
    A = np.sqrt((x2 - x1)**2 + (y2 - y1)**2 + (z2 - z1)**2)
    B = np.sqrt((x3 - x2)**2 + (y3 - y2)**2 + (z3 - z2)**2)
    # 始点と終点の長さ
    C = np.sqrt((x3 - x1)**2 + (y3 - y1)**2 + (z3 - z1)**2)
    # 余弦定理を用いて作成した角度
    cos = (C**2 - A**2 - B**2) / (2.0 * A * B)

    if cos >= 1:
        theta = 0.0
    elif cos <= -1:
        theta = np.pi
    else:
        theta = np.arccos(cos)

    deg = rotation * theta * 180 / np.pi
    
    # 欠損値の場合、前回の値を参照
    if np.isnan(deg):
        if temp_deg is not np.isnan(deg):
            deg = temp_deg
        else:
            deg = 0.0  # 初回の場合、デフォルト値を設定
            
    # 前回の値を更新
    temp_deg = deg

    return deg
    
def Sholuder_IK(X2, Y2, Z2, X3, Y3, Z3):
    
    q1, q2 = symbols('q1 q2')

    r = np.array([X2 - X3, Y2 - Y3, Z2 - Z3])
    norm_r = np.linalg.norm(r)

    e = r / norm_r
    e_x, e_y, e_z = e

    q2 = np.arcsin(-e_y)
    q1 = np.arctan2(e_x, e_z)

    theta1 = np.rad2deg(q1)
    theta2 = np.rad2deg(q2)

    return theta1, theta2

#検出された部位に対して,長方形で囲むための関数
#image:対象の座標,x:原点のx座標,y:原点のy座標,w:長方形の幅,h:長方形の高さ
#@jit(nopython=True, cache=True)

def draw_rectangle(image, x, y, w, h):
        # 検出された物体に対して長方形を描画
        Box = cv2.rectangle(image, [int(x), int(y)], [int(w), int(h)], color = (0, 0, 0), thickness = 2)
        return Box
        
#q1はピッチ軸周り,q2はロール軸周り
def Sholuder_IK(X1, Y1, Z1, X2, Y2, Z2):
    
    q1, q2 = symbols('q1 q2')

    r = np.array([X1 - X2, Y1 - Y2, Z1 - Z2])
    norm_r = np.linalg.norm(r)

    e = r / norm_r
    e_x, e_y, e_z = e

    q1 = np.arctan2(e_x, e_z)
    q2 = -np.arcsin(-e_y)
    
    theta1 = np.rad2deg(q1)
    theta2 = np.rad2deg(q2)

    return theta1, theta2

def test(point_neck,point_RSho):
    test = point_RSho.x - point_neck.x
    
    return test
    
def Sholuder_IK_Mark2(X1, Y1, Z1, X2, Y2, Z2):
    
    q1, q2 = symbols('q1 q2')

    r = np.array([X1 - X2, Y1 - Y2, Z1 - Z2])
    norm_r = np.linalg.norm(r)

    e = r / norm_r
    e_x, e_y, e_z = e

    q1 = np.arctan2(e_x, e_z)
    
    y = -np.arcsin(-e_y)
    
    q2 = np.arctan2(y,np.sqrt(1 - np.square(y)))
    
    theta1 = np.rad2deg(q1)
    theta2 = np.rad2deg(q2)

    return theta1, theta2


def get_right_arm_angles(X1,Y1,Z1,X2,Y2,Z2,X3,Y3,Z3):
    # openpose_outputはOpenPoseの出力と仮定
    # 右肩の座標を取得
    right_shoulder = np.array([X1, Y1, Z1])
    # 右肘の座標を取得
    right_elbow = np.array([X2, Y2, Z2])
    # 右手首の座標を取得
    right_wrist = np.array([X3, Y3, Z3])


    # 右肩と右肘のベクトルを計算
    shoulder_to_elbow = right_elbow - right_shoulder
    # 右肩と右手首のベクトルを計算
    shoulder_to_wrist = right_wrist - right_shoulder

    # 右肩のピッチ軸とロール軸の回転角度を計算します。これは、肩から肘へのベクトルを使用しています。
    r1 = R.from_rotvec(shoulder_to_elbow)
    pitch_angle, roll_angle, _ = r1.as_euler('xyz', degrees=True)

    # 右肘のヨー軸とピッチ軸の回転角度を計算します。これは、肩から手首へのベクトルを使用しています。
    r2 = R.from_rotvec(shoulder_to_wrist)
    yaw_angle, elbow_pitch_angle, _ = r2.as_euler('xyz', degrees=True)

    # 角度を返す
    #return r1,r2,pitch_angle, roll_angle, yaw_angle, elbow_pitch_angle
    return right_shoulder

def euler_to_quaternion(pitch, roll):
    """
    オイラー角をクォータニオンに変換する関数

    Parameters:
    - pitch: Y軸回りの回転角度 (ラジアン)
    - roll: X軸回りの回転角度 (ラジアン)

    Returns:
    - quaternion: 変換されたクォータニオン [w, x, y, z]
    """    
    r = Rotation.from_euler('yx', [pitch, roll], degrees=False)
    quaternion = r.as_quat()
    return quaternion
    

def get_sholuder_IK_EQE(p_Sholuder,p_Elbow,q_org):
    
    
    #r = np.array([point_1[0] - point_2[0], point_1[1] - point_2[1], point_1[2] - point_2[2]])
    r = p_Sholuder - p_Elbow
    norm_r = np.linalg.norm(r)
    
    e = r /norm_r
    
    e_x, e_y, e_z = e
    
    q2a = np.arcsin(e_y)
    
    if q2a > 0:
        q2b = np.pi - q2a
    else:
        #コメントアウトの方な気がする.........
        #q2b = -np.pi + q2a
        q2b = -np.pi - q2a

    if np.abs(q2a - np.radians(q_org[1])) < np.abs(q2b - np.radians(q_org[1])):
        q2 = q2a
    else:
        q2 = q2b

    c2 = np.cos(q2)

    if np.abs(c2) > np.deg2rad(5.0): #np.finfo(float).eps
        q1 = np.arctan2( -e_x / c2, e_z / c2)
    else:
        q1 = np.radians(q_org[0])
    
    q = tf.transformations.quaternion_from_euler(q2, q1, 0)
    
    e = tf.transformations.euler_from_quaternion((q[0], q[1], q[2], q[3]))
    
    theta1 = np.rad2deg(e[1])
    theta2 = np.rad2deg(e[0])
    
    return theta1,theta2

