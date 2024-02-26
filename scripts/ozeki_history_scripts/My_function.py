import numpy as np
import cv2
from numba import jit


"""
Make My function
"""

#@jit(nopython=True)

def pixel_to_robot_coordinates(image_width, image_height, robot_width, robot_height, pixel):
    
    # 画像座標系からロボット座標への変換行列
    # スクリーン座標系のZ軸の向きが逆転しているため、変換が必要。

    scale_x = robot_width / image_width
    scale_y = -robot_height / image_height

    # スクリーンの中心位置をロボット座標系の原点に合わせるための処理

    translation_x = robot_width / 2
    translation_y = robot_height / 2

    # スクリーン座標をロボット座標に変換
    # スクリーン座標のX軸はロボットのピッチ軸に該当し、Y軸はヨー軸に該当する。

    robot_y = pixel.x * scale_x + translation_x
    robot_z = pixel.y * scale_y + translation_y

    return robot_y, robot_z

#degの角度範囲は-15°~225°
#y0,z0は始点,y1,z1は始点の番号1つ前,y2,z2は始点の番号1つ後

#@jit(nopython=True)

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


#検出された部位に対して,長方形で囲むための関数
#image:対象の座標,x:原点のx座標,y:原点のy座標,w:長方形の幅,h:長方形の高さ
def draw_rectangle(image, x, y, w, h):
        # 検出された物体に対して長方形を描画
        Box = cv2.rectangle(image, [int(x), int(y)], [int(w), int(h)], color = (0, 0, 0), thickness = 2)
        return Box