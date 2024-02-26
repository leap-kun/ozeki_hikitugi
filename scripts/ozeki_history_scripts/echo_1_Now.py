#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
from ros_openpose.msg import Frame
from ros_openpose.msg import Pixel
from ros_openpose.msg import BodyPart
#from ros_opsepose.msg import Persons
import cv2
import sys
import message_filters
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import numpy as np
from std_msgs.msg import Int16
from std_msgs.msg import Float32
from sensor_msgs.msg import JointState
"""
#############################左##########################################
0:頭ヨー,1:左肩ピッチ,2:左肩ロール,3;左肘ロール,4:左肘ピッチ
5:左股ロール,6:左股ロール,7:左股ピッチ,8:左膝ピッチ,9:左足首ピッチ,10:左足首ロール
"""
trim_l = np.array([0,-1350,-2580,0,2650,0,70,-25,-1600,-600,20])
cw_l = np.array([1,-1,1,1,1,-1,-1,-1,1,1,1])
"""
#############################右##########################################
0:腰ヨー,1:右肩ピッチ,2:右肩ロール,3;右肘ロール,4:右肘ピッチ
5:右股ロール,6:右股ロール,7:右股ピッチ,8:右膝ピッチ,9:右足首ピッチ,10:右足首ロール
"""
trim_r = np.array([0,1350,2700,0,-2650,0,-30,-60,1600,700,-20])
cw_r = np.array([1,1,-1,-1,-1,1,1,1,-1,-1,-1])

def image_to_robot_coordinates(image_width, image_height, robot_width, robot_height, image_x, image_y):
    
    # 画像座標系からロボット座標への変換行列
    # スクリーン座標系のZ軸の向きが逆転しているため、変換が必要。
    scale_x = robot_width / image_width
    scale_y = -robot_height / image_height
    # スクリーンの中心位置をロボット座標系の原点に合わせるための処理
    translation_x = robot_width / 2
    translation_y = robot_height / 2
    # スクリーン座標をロボット座標に変換
    # スクリーン座標のX軸はロボットのピッチ軸に該当し、Y軸はヨー軸に該当する。
    robot_y = image_x * scale_x + translation_x
    robot_z = image_y * scale_y + translation_y
    return robot_y, robot_z

#degの角度範囲は-15°~225°
#y0,z0は始点,y1,z1は始点の番号1つ前,y2,z2は始点の番号1つ後
def inner(y0, y1, y2, z0, z1, z2):
    
    #y0,z0 = 右肩, y1,z1 = 首,y2,z2 = 右肘
    vec1 = [y1 - y0, z1 - z0]
    vec2 = [y2 - y0, z2 - z0]
        
    inner = np.inner(vec1,vec2)
    norm = np.linalg.norm(vec1) * np.linalg.norm(vec2)
    
    cos_theta = inner / norm
    theta = np.rad2deg(np.arccos(np.clip(cos_theta, -1.0, 1.0)))
    
    return theta

#degの角度範囲は-15°~225°
#y0,z0は始点,y1,z1は始点の番号1つ前,y2,z2は始点の番号1つ後
#atan2で角度を求めている。

def atan2_deg(y0, y1, y2, z0, z1, z2):
    
    # ベクトル1とベクトル2のなす角を計算します
    angle1 = np.arctan2(z1 - z0, y1 - y0)
    
    # ベクトル2とベクトル3のなす角を計算します
    angle2 = np.arctan2(z2 - z0, y2 - y0)
    
    # 三点のなす角を計算します
    total_angle = angle2 - angle1
    
    if total_angle < 0:
        total_angle += 2 * np.pi
    
    degree = total_angle * 180 / np.pi

    return degree

#普段の座標系からKRSモーターの座標系に変換(-135°~135の変化(0°原点))
#第一引数:innerから生成された角度
#if文の表記は,Motorの安全確保を図っている。
def ChangeDeg(Motor_deg):
    
    Motor_deg = -(Motor_deg - 90)
    
    if(Motor_deg > 89):
        Motor_deg = 89
    elif(Motor_deg < -89):
        Motor_deg = -89
    
    return Motor_deg


#角度からHTH値に変換
#第一引数:degree値,第二引数;回転方向
#deg_argにはChangeDegの関数で回転方向を修正したものを入れる
#4000(一象限当たりのmotorの変化分) / 135(一象限当たりの角度の最大変化) = 29.629..............
#if文はモーターの安全確保を図っている。
def ChDegtoHTH(trim_arg,deg_arg,cw_arg):
    
    HTH = 7500 + trim_arg + (deg_arg * 29.6296 * cw_arg)
    
    if(HTH > 11500):
        HTH = 11500
    elif(HTH < 3500):
        HTH = 3500
           
    return HTH

class depth_estimater:
    queue_size = 100
    fps = 10.
    delay = 0.5
    def __init__(self):
        rospy.init_node('echo', anonymous=False)
        self.OpenPose_HTH = rospy.Publisher('ros_openpose_HTH', Int16, queue_size=10,latch = True)
        #self.OpenPose_theta = rospy.Publisher('theta', Float32 ,queue_size = 10)
        self.bridge = CvBridge()
        frame_topic = rospy.get_param('~pub_topic')
        sub_rgb = message_filters.Subscriber("/usb_cam/image_raw", Image)
        sub_pose = message_filters.Subscriber(frame_topic, Frame)
        self.mf = message_filters.ApproximateTimeSynchronizer([sub_pose, sub_rgb], 100, 0.5)
        self.mf.registerCallback(self.ImageCallback)
    def ImageCallback(self, pose_data, rgb_data):
        try:
            color_image = self.bridge.imgmsg_to_cv2(rgb_data, 'rgb8')
            color_image = cv2.resize(color_image, (int(color_image.shape[1]*0.25), int(color_image.shape[0]*0.25)))
            #bodyPart.pixelはFrame.msgから引っ張ってくる。(三次元座標をやりたかったらbodyPart.points?????)
            #person,bodypartは変数
            #pose_data.person:人数のリスト,person.bodyParts:座標情報のリスト(pixelの情報)
            text = [bodyPart.pixel for person in np.array(pose_data.persons) for bodyPart in np.array(person.bodyParts)]
        except CvBridgeError as e:
            rospy.logerr(e)
        try:
            
            #一人分の座標を取り出すための処理(無理やり)
            """ OpenPose skeleton dictionary
            {0, "Nose"}, {13, "LKnee"}
            {1, "Neck"}, {14, "LAnkle"}
            {2, "RShoulder"}, {15, "REye"}
            {3, "RElbow"}, {16, "LEye"}
            {4, "RWrist"}, {17, "REar"}
            {5, "LShoulder"}, {18, "LEar"}
            {6, "LElbow"}, {19, "LBigToe"}
            {7, "LWrist"}, {20, "LSmallToe"}
            {8, "MidHip"}, {21, "LHeel"}
            {9, "RHip"}, {22, "RBigToe"}
            {10, "RKnee"}, {23, "RSmallToe"}
            {11, "RAnkle"}, {24, "RHeel"}
            {12, "LHip"}, {25, "Background"}
            """
            if len(pose_data.persons) == 1:
                image_x_Neck = int(text[1].x)
                image_y_Neck = int(text[1].y)
                image_x_RShoulder = int(text[2].x)
                image_y_RShoulder = int(text[2].y)
                image_x_RElbow = int(text[3].x)
                image_y_RElbow = int(text[3].y)
                image_x_RWrist = int(text[4].x)
                image_y_RWrist = int(text[4].y)
                image_x_LShoulder = int(text[5].x)
                image_y_LShoulder = int(text[5].y)
                image_x_LElbow = int(text[6].x)
                image_y_LElbow = int(text[6].y)
                
                
            elif len(pose_data.persons) == 0:
                #誰もいない場合
                print("No Person.........")
                
            else:
                #二人以上の場合
                image_x_Neck = int(text[1].x.pose_data.persons[0])
                image_y_Neck = int(text[1].y.pose_data.persons[0])
                image_x_RShoulder = int(text[2].x.pose_data.persons[0])
                image_y_RShoulder = int(text[2].y.pose_data.persons[0])
                image_x_RElbow = int(text[2].x.pose_data.persons[0])
                image_y_RElbow = int(text[2].y.pose_data.persons[0])
                print("Too Person!!!!!!!!!!!!!")
            
            # ロボット座標への変換
            robot_width = 525.8  # KHR-3HVのY軸方向の幅(両腕:525.8mm)
            robot_height = 401.1  #KHR-3HVのZ軸方向の高さ(身長:401.1mm)
            image_width = 640  # 画像の幅
            image_height = 480  # 画像の高さ
            
            robot_y0, robot_z0 = image_to_robot_coordinates(image_width, image_height, robot_width, robot_height, image_x_RShoulder, image_y_RShoulder)
            #print("右肩 Y: {}, Z: {}".format(robot_y0, robot_z0))
            
            #首
            robot_y1, robot_z1 = image_to_robot_coordinates(image_width, image_height, robot_width, robot_height, image_x_Neck, image_y_Neck)
            #print("首 Y: {}, Z: {}".format(robot_y1, robot_z1))
            
            #右肘
            robot_y2, robot_z2 = image_to_robot_coordinates(image_width, image_height, robot_width, robot_height, image_x_RElbow, image_y_RElbow)
            #print("右肘 Y: {}, Z: {}".format(robot_y2, robot_z2))
            deg_R2 = inner(robot_y0,robot_y1,robot_y2,robot_z0,robot_z1,robot_z2)
            
            conv_deg_R2 = ChangeDeg(deg_R2)
            
            #今回はint32で通信しているため,intにキャストしないと駄目^^;
            HTH_R_2 = int(ChDegtoHTH(trim_r[2],conv_deg_R2,cw_r[2]))
            
            #print("KRS:{}".format(KRS))
            
            print("deg:{0},conv_deg:{1},KRS:{2}".format(int(deg_R2),int(conv_deg_R2),HTH_R_2))
            
            #KRS値をパブリッシュ
            self.OpenPose_HTH.publish(HTH_R_2)
            #rospy,sleepは0.1~0.2の間が好ましい。
            rospy.sleep(0.1)
            #rospy.Rate.sleep()
            
        except:
            pass
if __name__ == '__main__':
    try:
        de = depth_estimater()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass