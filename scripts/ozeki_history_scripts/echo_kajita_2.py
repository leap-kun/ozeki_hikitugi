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
from std_msgs.msg import Int16MultiArray
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import JointState
import rosbag
from datetime import datetime
import csv

#以下がimportFile

#Motorの動作関数をまとめたファイル
import Move_Motor as MM
#画像の原点調整をして,KRS座標系に変換したもの
import ChangeToDeg as Ch



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

def atan2_deg(y0, y1, y2, z0, z1, z2):
    
    #y0,z0 = 右肩, y1,z1 = 首,y2,z2 = 右肘
    
    # ベクトル1とベクトル2のなす角を計算します
    angle1 = np.arctan2(z1 - z0, y1 - y0)
    
    # ベクトル2とベクトル3のなす角を計算します
    angle2 = np.arctan2(z2 - z0, y2 - y0)
    
    # 三点のなす角を計算します
    total_angle = angle1 - angle2
    
    
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


class depth_estimater:
    queue_size = 100
    fps = 10.
    delay = 0.5
    

    def __init__(self):
        rospy.init_node('echo', anonymous=False)
        
        #Use Myself RosTopic
        self.Pub_deg = rospy.Publisher('Pub_deg' , Float32MultiArray , queue_size = 10, latch = True)
        self.Conv_deg = rospy.Publisher('Conv_deg', Float32MultiArray, queue_size = 10,latch = True)
        self.Pub_Score = rospy.Publisher('Pub_Score',BodyPart ,queue_size = 10 ,latch = True)
        #画像系のトピック
        
        self.bridge = CvBridge()
        frame_topic = rospy.get_param('~pub_topic')
        sub_rgb = message_filters.Subscriber("/usb_cam/image_raw", Image)
        sub_pose = message_filters.Subscriber(frame_topic, Frame)
        self.mf = message_filters.ApproximateTimeSynchronizer([sub_pose, sub_rgb], 100, 0.5)
        self.mf.registerCallback(self.ImageCallback)
        
        
        #rosbagファイルの作成
        
        now = datetime.now()#現在の日時を習得
        date_string = now.strftime("%Y_%m_%d_%H:%M")  # 日付を指定の形式に変換

        #thetaのpath
        file_path_theta = '/home/kajilab/catkin_ws/src/ros_openpose/rosbag_file/rosbag_deg'
        file_name_theta = f'Pub_deg_{date_string}.bag' # ファイル名に日付を追加
        self.DegBag = rosbag.Bag(file_path_theta + '/' + file_name_theta,'w') # fileに書き込み
        
        #theta変更後の角度のpath
        file_path_conv = '/home/kajilab/catkin_ws/src/ros_openpose/rosbag_file/rosbag_conv_theta'
        file_name_conv = f'Pub_Conv_theta_{date_string}.bag' # ファイル名に日付を追加
        self.convBag = rosbag.Bag(file_path_conv + '/' + file_name_conv,'w') # fileに書き込み
        
        """
        self.csv_file_HTH = open('/home/kajilab/catkin_ws/src/ros_openpose/csv_file/csv_HTH.csv', 'w')
        self.csv_writer_HTH = csv.writer(self.csv_file_HTH)
        self.csv_writer_HTH.writerow(['HTH_R_2', 'HTH_L_2'])

        self.csv_file_theta = open('/home/kajilab/catkin_ws/src/ros_openpose/csv_file/csv_theta.csv', 'w')
        self.csv_writer_theta = csv.writer(self.csv_file_theta)
        self.csv_writer_theta.writerow(['theta_R_2', 'theta_L_2'])

        self.csv_file_conv = open('/home/kajilab/catkin_ws/src/ros_openpose/csv_file/csv_conv_theta.csv', 'w')
        self.csv_writer_conv = csv.writer(self.csv_file_conv)
        self.csv_writer_conv.writerow(['conv_deg_R_2', 'conv_deg_L_2'])
        """

    def ImageCallback(self, pose_data, rgb_data):
        try:
            color_image = self.bridge.imgmsg_to_cv2(rgb_data, 'rgb8')
            color_image = cv2.resize(color_image, (int(color_image.shape[1]*0.25), int(color_image.shape[0]*0.25)))
            #bodyPart.pixelはFrame.msgから引っ張ってくる。(三次元座標をやりたかったらbodyPart.points?????)
            #person,bodypartは変数
            #pose_data.person:人数のリスト,person.bodyParts:座標情報のリスト(pixelの情報)
            text = [bodyPart.pixel for person in np.array(pose_data.persons) for bodyPart in np.array(person.bodyParts)]
            #test = [bodyPart.pixel for person in pose_data.persons[0] for bodyPart in person.bodyParts]
            #test = [bodyPart.pixel for bodyPart in pose_data.persons[0].bodyParts]
            #test = [bodyPart.pixel for bodyPart in pose_data.persons[0].bodyParts]


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
            if len(pose_data.persons) != 0:
                
                """
                image_width = 640 = 画像の幅である(認識範囲は280~360に設定)
                ここではif文で人数の認識及び画像内に今存在している座標に対して,targetを設定
                targetは人数である.(例:target[0] = 一人目, target[1] = 二人目)
                bodyParts[0]は鼻であり,ここでは,鼻の画像座標がx座標のどこにいるかで何人目かを絞っている.
                今現段階では,鼻のx座標が真ん中にいるように設定 = KHRの操縦者が真ん中にいないといけない
                """
                
                if len(pose_data.persons) == 1:
                    target = 0
                elif pose_data.persons[0].bodyParts[0].pixel.x > 280  and pose_data.persons[0].bodyParts[0].pixel.x < 360:
                    target = 0
                elif pose_data.persons[1].bodyParts[0].pixel.x > 280 and pose_data.persons[1].bodyParts[0].pixel.x < 360:
                    target = 1
                elif pose_data.persons[2].bodyParts[0].pixel.x > 280 and pose_data.persons[2].bodyParts[0].pixel.x < 360:
                    target = 2
                elif pose_data.persons[3].bodyParts[0].pixel.x > 280 and pose_data.persons[3].bodyParts[0].pixel.x < 360:
                    target = 3
                
                #上半身
                
                image_x_Neck = int(pose_data.persons[target].bodyParts[1].pixel.x) 
                image_y_Neck = int(pose_data.persons[target].bodyParts[1].pixel.y) 
                image_x_RShoulder = int(pose_data.persons[target].bodyParts[2].pixel.x) 
                image_y_RShoulder = int(pose_data.persons[target].bodyParts[2].pixel.y) 
                image_x_RElbow = int(pose_data.persons[target].bodyParts[3].pixel.x) 
                image_y_RElbow = int(pose_data.persons[target].bodyParts[3].pixel.y) 
                image_x_RWrist = int(pose_data.persons[target].bodyParts[4].pixel.x) 
                image_y_RWrist = int(pose_data.persons[target].bodyParts[4].pixel.y) 
                image_x_LShoulder = int(pose_data.persons[target].bodyParts[5].pixel.x) 
                image_y_LShoulder = int(pose_data.persons[target].bodyParts[5].pixel.y) 
                image_x_LElbow = int(pose_data.persons[target].bodyParts[6].pixel.x) 
                image_y_LElbow = int(pose_data.persons[target].bodyParts[6].pixel.y) 
                
                """
                #下半身
                
                image_x_MidHip = int(pose_data.persons[target].bodyParts[8].pixel.x) 
                image_y_MidHip = int(pose_data.persons[target].bodyParts[8].pixel.y) 
                image_x_RHip = int(pose_data.persons[target].bodyParts[9].pixel.x) 
                image_y_RHip = int(pose_data.persons[target].bodyParts[9].pixel.y)
                image_x_RKnee = int(pose_data.persons[target].bodyParts[10].pixel.x) 
                image_y_RKnee = int(pose_data.persons[target].bodyParts[10].pixel.y) 
                
                image_x_LHip = int(text[12].x)
                image_y_LHip = int(text[12].y)
                image_x_LKnee = int(text[13].x)
                image_y_LKnee = int(text[13].y)
                """
                
                
                
                """
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
                
                #下半身
                
                image_x_MidHip = int(text[8].x)
                image_y_MidHip = int(text[8].y)
                image_x_RHip = int(text[9].x)
                image_y_RHip = int(text[9].y)
                image_x_RKnee = int(text[10].x)
                image_y_RKnee = int(text[10].y)
                
                image_x_LHip = int(text[12].x)
                image_y_LHip = int(text[12].y)
                image_x_LKnee = int(text[13].x)
                image_y_LKnee = int(text[13].y)
                """
                
                
                
            else:
                #誰もいない場合
                print("No Person.........")
                
                """
                #print(pose_data)
                print("Too people!!!!!!!!!!!!!")
                #二人以上の場合
                image_x_Neck = int(test[1].x)
                image_y_Neck = int(test[1].y)
                image_x_RShoulder = int(test[2].x)
                image_y_RShoulder = int(test[2].y)
                image_x_RElbow = int(test[3].x)
                image_y_RElbow = int(test[3].y)
                image_x_RWrist = int(test[4].x)
                image_y_RWrist = int(test[4].y)
                image_x_LShoulder = int(test[5].x)
                image_y_LShoulder = int(test[5].y)
                image_x_LElbow = int(test[6].x)
                image_y_LElbow = int(test[6].y)
            
                
                print(pose_data.persons[0].bodyParts[1].pixel.x)
                
                image_x_Neck = int(pose_data.persons[0].bodyParts[1].pixel.x)
                image_y_Neck = int(pose_data.persons[0].bodyParts[1].pixel.y)
                image_x_RShoulder = int(pose_data.persons[0].bodyParts[2].pixel.x)
                image_y_RShoulder = int(pose_data.persons[0].bodyParts[2].pixel.y)
                image_x_RElbow = int(pose_data.persons[0].bodyParts[3].pixel.x)
                image_y_RElbow = int(pose_data.persons[0].bodyParts[3].pixel.y)
                """

            
            
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
            
            #左肩
            robot_yL5,robot_zL5 = image_to_robot_coordinates(image_width, image_height, robot_width, robot_height, image_x_LShoulder, image_y_LShoulder)
            
            #右肩
            robot_yL6,robot_zL6 = image_to_robot_coordinates(image_width, image_height, robot_width, robot_height, image_x_LElbow, image_y_LElbow)
            
            """
            #中尻
            robot_y8,robot_z8 = image_to_robot_coordinates(image_width, image_height, robot_width, robot_height, image_x_MidHip, image_y_MidHip)
            
            #右尻
            robot_y9,robot_z9 = image_to_robot_coordinates(image_width, image_height, robot_width, robot_height, image_x_RHip, image_y_RHip)
            
            #右膝
            robot_y10,robot_z10 = image_to_robot_coordinates(image_width, image_height, robot_width, robot_height, image_x_RKnee, image_y_RKnee)
            
            #左尻
            robot_y12,robot_z12 = image_to_robot_coordinates(image_width, image_height, robot_width, robot_height, image_x_LHip, image_y_LHip)
            
            #左膝
            robot_y13,robot_z13 = image_to_robot_coordinates(image_width, image_height, robot_width, robot_height, image_x_LKnee, image_y_LKnee)
            """


            deg_R2 = atan2_deg(robot_y0,robot_y1,robot_y2,robot_z0,robot_z1,robot_z2)
            deg_L2 = atan2_deg(robot_yL5,robot_yL6,robot_y1,robot_zL5,robot_zL6,robot_z1)
            

            Deg = np.array([deg_R2, deg_L2])
            Pub_Deg = Float32MultiArray(data = Deg)
            
            #角度変換
            conv_deg_R2 = Ch.ChangeDegR2(deg_R2)
            conv_deg_L2 = Ch.ChangeDegL2(deg_L2)
            
            #変更した角度を格納するための空の配列
            conv = np.array([conv_deg_R2, conv_deg_L2])
            Pub_conv = Float32MultiArray(data = conv)
            
            #print("deg:{0},conv_deg:{1},R_2:{2}".format(int(deg_R2),int(conv_deg_R2),HTH_R_2))
            #print("deg:{0},conv_deg:{1},L_2:{2}".format(int(deg_R2),int(conv_deg_R2),HTH_L_2))
            print("Conv_degR:{0},Conv_degL:{1}".format(int(conv_deg_R2),int(conv_deg_L2)))
            
            #print("Conv_degLLLLLLLL:{0}".format(int((conv_deg_L2))))
            
            #bagファイルにデータを書き込む(theta)
            self.DegBag.write('deg_array', Pub_Deg, rospy.Time.now())

             #bagファイルにデータを書き込む(conv_theta)
            self.convBag.write('conv_deg_array',Pub_conv,rospy.Time.now())

            
            #Pub_conv値をパブリッシュ
            self.Conv_deg.publish(Pub_conv)

            
        except:
            pass
    
    #bagファイルを閉じる処理
    def __del__(self):
            # ノードが終了するときにrosbagファイルを閉じる
        self.DegBag.close()
        self.convBag.close()
        """
        self.csv_file_HTH.close()
        self.csv_file_theta.close()
        self.csv_file_conv.close()
        """
        
if __name__ == '__main__':
    try:
        #10秒間起動を遅らせる
        rospy.loginfo("Start OpenPose Sync")
        de = depth_estimater()
        rospy.spin()
        """
        # 10秒間待機
        rospy.sleep(10.0)
        # rospy.spin()の前にshutdownを呼び出してノードを終了する
        sys.exit()
        """
        
    except rospy.ROSInterruptException:
        pass