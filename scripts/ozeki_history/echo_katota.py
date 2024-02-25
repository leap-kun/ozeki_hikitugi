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

#画像の原点調整をして,KRS座標系に変換したもの
import ChangeToDeg as Ch

#atan2で角度計算
from My_function import atan2_deg
#スクリーン座標からロボット座標に変換
from My_function import pixel_to_robot_coordinates
#長方形(バウンディングボックスモドキ)の生成
from My_function import draw_rectangle


class depth_estimater:
    queue_size = 100
    fps = 10.
    delay = 0.5
    

    def __init__(self):
        
        #Use Myself RosTopic
        self.Pub_deg = rospy.Publisher('Pub_deg' , Float32MultiArray , queue_size = 10, latch = True)
        self.Pub_Conv_deg = rospy.Publisher('Pub_Conv_deg', Float32MultiArray, queue_size = 10,latch = True)
        self.Pub_Score = rospy.Publisher('Pub_Score',Float32MultiArray ,queue_size = 10 ,latch = True)
        
        #画像系のトピック
        
        self.bridge = CvBridge()
        frame_topic = rospy.get_param('~pub_topic')
        sub_rgb = message_filters.Subscriber("/usb_cam/image_raw", Image)
        sub_pose = message_filters.Subscriber(frame_topic, Frame)
        self.mf = message_filters.ApproximateTimeSynchronizer([sub_pose, sub_rgb], 100, 0.5)
        self.mf.registerCallback(self.ImageCallback)
        
        # ロボット座標への変換
        self.robot_width = 525.8  # KHR-3HVのY軸方向の幅(両腕:525.8mm)
        self.robot_height = 401.1  #KHR-3HVのZ軸方向の高さ(身長:401.1mm)
        self.image_width = 640  # 画像の幅
        self.image_height = 480  # 画像の高さ
            
        
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
        
        #各部位の信頼度(Score)
        file_path_theta = '/home/kajilab/catkin_ws/src/ros_openpose/rosbag_file/rosbag_score'
        file_name_theta = f'Pub_score_{date_string}.bag' # ファイル名に日付を追加
        self.scoreBag = rosbag.Bag(file_path_theta + '/' + file_name_theta,'w') # fileに書き込み

        


    def ImageCallback(self, pose_data, rgb_data):
        try:
            color_image = self.bridge.imgmsg_to_cv2(rgb_data, 'bgr8')
            color_image = cv2.resize(color_image, (int(color_image.shape[1]*0.25), int(color_image.shape[0]*0.25)))
            #bodyPart.pixelはFrame.msgから引っ張ってくる。(三次元座標をやりたかったらbodyPart.points?????)
            #person,bodypartは変数
            
            #pose_data.person:人数のリスト,person.bodyParts:座標情報のリスト(pixelの情報)
            text = [bodyPart.pixel for person in np.array(pose_data.persons) for bodyPart in np.array(person.bodyParts)]
            get_score = [bodyPart.score for person in np.array(pose_data.persons) for bodyPart in np.array(person.bodyParts)]
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
                self.image_width = 640 = 画像の幅であるself.(認識範囲は280~360self.に設定)
                ここではif文で人数の認識及び画像内に今存在している座標に対して,targetを設定
                targetは人数である.(例:target[0] = 一人目, target[1] = 二人目)
                bodyParts[0]は鼻であり,ここでは,鼻の画像座標がx座標のどこにいるかで何人目かを絞っている.
                今現段階では,鼻のx座標が真ん中にいるように設定 = KHRの操縦者が真ん中にいないといけない
                """
                
                person_x = np.array([])
                for n in range(len(pose_data.persons)):
                    error_x = abs(pose_data.persons[n].bodyParts[0].pixel.x - 320)
                    person_x = np.append(person_x,error_x)
                
                target = np.argmin(person_x)
                
                
                
                #顔
                pixel_REye = pose_data.persons[target].bodyParts[15].pixel
                
                #上半身
                pixel_Neck = pose_data.persons[target].bodyParts[1].pixel
                pixel_RShoulder = pose_data.persons[target].bodyParts[2].pixel
                pixel_RElbow = pose_data.persons[target].bodyParts[3].pixel
                pixel_RWrist = pose_data.persons[target].bodyParts[4].pixel
                pixel_LShoulder = pose_data.persons[target].bodyParts[5].pixel
                pixel_LElbow = pose_data.persons[target].bodyParts[6].pixel 
                pixel_LWrist = pose_data.persons[target].bodyParts[7].pixel
                
                
                """
                #下半身
                
                pixel_MidHip = pose_data.persons[target].bodyParts[8].pixel 
                pixel_RHip = pose_data.persons[target].bodyParts[9].pixel
                pixel_RKnee = pose_data.persons[target].bodyParts[10].pixel 
                
                pixel_LHip = pose_data.persons[target].bodyParts[12].pixel 
                pixel_LKnee = pose_data.persons[target].bodyParts[13].pixel 
    
                """
                pixel_LBigToe = pose_data.persons[target].bodyParts[19].pixel
                
                
                #長方形の描画
                #draw_rectangle(color_image,image_x_RWrist,image_y_REye,image_x_LWrist,image_y_LBigToe)
                
                #draw_rectangle(color_image, pixel_RWrist.x, pixel_REye.y, pixel_LWrist.x, pixel_LWrist.y)
                
                draw_rectangle(color_image, 100, 200, 200, 300)


            
            else:
                #誰もいない場合
                print("No Person.........")
            
            
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
            

            
            robot_y0, robot_z0 = pixel_to_robot_coordinates(self.image_width, self.image_height, self.robot_width, self.robot_height, pixel_RShoulder)
            #print("右肩 Y: {}, Z: {}".format(robot_y0, robot_z0))
            
            #首
            robot_y1, robot_z1 = pixel_to_robot_coordinates(self.image_width, self.image_height, self.robot_width, self.robot_height, pixel_Neck)
            #print("首 Y: {}, Z: {}".format(robot_y1, robot_z1))
            
            #右肘
            robot_y2, robot_z2 = pixel_to_robot_coordinates(self.image_width, self.image_height, self.robot_width, self.robot_height, pixel_RElbow)
            #print("右肘 Y: {}, Z: {}".format(robot_y2, robot_z2))
            
            #左肩
            robot_yL5,robot_zL5 = pixel_to_robot_coordinates(self.image_width, self.image_height, self.robot_width, self.robot_height, pixel_LShoulder)
            
            #右肩
            robot_yL6,robot_zL6 = pixel_to_robot_coordinates(self.image_width, self.image_height, self.robot_width, self.robot_height, pixel_LElbow)
            
            """
            #中尻
            robot_y8,robot_z8 = My.image_to_robot_coordinates(self.image_width, self.image_height, self.robot_width, self.robot_height, pixel_MidHip)
            
            #右尻
            robot_y9,robot_z9 = My.image_to_robot_coordinates(self.image_width, self.image_height, self.robot_width, self.robot_height, pixel_RHip)
            
            #右膝
            robot_y10,robot_z10 = My.image_to_robot_coordinates(self.image_width, self.image_height, self.robot_width, self.robot_height, pixel_RKnee)
            
            #左尻
            robot_y12,robot_z12 = My.image_to_robot_coordinates(self.image_width, self.image_height, self.robot_width, self.robot_height, pixel_LHip)
            
            #左膝
            robot_y13,robot_z13 = My.image_to_robot_coordinates(self.image_width, self.image_height, self.robot_width, self.robot_height, pixel_LKnee)
            """
            

            deg_R2 = atan2_deg(robot_y0,robot_y1,robot_y2,robot_z0,robot_z1,robot_z2)
            deg_L2 = atan2_deg(robot_yL5,robot_yL6,robot_y1,robot_zL5,robot_zL6,robot_z1)
            

            Deg = np.array([deg_R2, deg_L2])
            Pub_Deg = Float32MultiArray(data = Deg)
            
            #角度変換
            Pub_conv_deg_R2 = Ch.ChangeDegR2(deg_R2)
            Pub_conv_deg_L2 = Ch.ChangeDegL2(deg_L2)
            
            #変更した角度を格納するための空の配列
            conv = np.array([Pub_conv_deg_R2, Pub_conv_deg_L2])
            Pub_conv = Float32MultiArray(data = conv)
            
            #各部位の信頼度(score)を格納
            
            score = get_score
            Pub_score = Float32MultiArray(data = score)
            
            #print("deg:{0},Pub_conv_deg:{1},R_2:{2}".format(int(deg_R2),int(Pub_conv_deg_R2),HTH_R_2))
            #print("deg:{0},Pub_conv_deg:{1},L_2:{2}".format(int(deg_R2),int(Pub_conv_deg_R2),HTH_L_2))
            print("Pub_Conv_degR:{0},Pub_Conv_degL:{1}".format(int(Pub_conv_deg_R2),int(Pub_conv_deg_L2)))
            
            
            #bagファイルにデータを書き込む(theta)
            self.DegBag.write('deg_array', Pub_Deg, rospy.Time.now())

             #bagファイルにデータを書き込む(conv_theta)
            self.convBag.write('conv_deg_array',Pub_conv,rospy.Time.now())
            
            #bagファイルにデータを書き込む(score)
            self.convBag.write('score_array',Pub_score,rospy.Time.now())
            
            #Pub_conv値をパブリッシュ
            self.Pub_Conv_deg.publish(Pub_conv)

            
        except:
            pass
    
    #bagファイルを閉じる処理
    def __del__(self):
            # ノードが終了するときにrosbagファイルを閉じる
        self.DegBag.close()
        self.convBag.close()
        self.scoreBag.close()
        
if __name__ == '__main__':
    try:
        #ノードの宣言
        rospy.init_node('echo', anonymous=False)
        #10秒間起動を遅らせる
        #rospy.sleep(10)
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