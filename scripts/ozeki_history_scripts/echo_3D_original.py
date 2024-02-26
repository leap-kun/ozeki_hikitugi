#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
from ros_openpose.msg import Frame
from ros_openpose.msg import BodyPart

#from ros_opsepose.msg import Persons
import cv2
import sys
import message_filters
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import numpy as np
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import JointState
import rosbag
from datetime import datetime
import csv
from geometry_msgs.msg import Point32
from scipy.spatial.transform import Rotation as R
from scipy.linalg import pinv
from scipy.signal import medfilt


#以下がimportFile

#publishする角度の制限
from My_function_3D import limit_deg
from My_function_3D import camera_to_robot_coordinates
from My_function_3D import replace_nan_with_previous
from My_function_3D import replace_nan_with_previous_ez
from My_function_3D import ChUnitVector
from My_function_3D import shoulder_ik_np
from My_function_3D import shoulder_ik_R

from Filter_3D import calculate_ema
from Filter_3D import apply_moving_average_3d
from Filter_3D import gaussian_filter
from Filter_3D import median1d
from Filter_3D import kalman_filter

class depth_estimater:
    queue_size = 100
    fps = 10.
    delay = 0.5
    

    def __init__(self):
        
        #Use Myself RosTopic
        self.Pub_deg = rospy.Publisher('Pub_deg' , Float32MultiArray , queue_size = 10, latch = True)
        self.Pub_Conv_deg = rospy.Publisher('Pub_Conv_deg', Float32MultiArray, queue_size = 10,latch = True)
        self.Pub_point = rospy.Publisher('Pub_point',Float32MultiArray ,queue_size = 10 ,latch = True)
        
        #画像系のトピック
        
        self.bridge = CvBridge()
        frame_topic = rospy.get_param('~pub_topic')
        #sub_rgb = message_filters.Subscriber("/usb_cam/image_raw", Image)
        sub_depth =  message_filters.Subscriber("/depth_to_rgb/image_raw",Image)
        print(sub_depth)
        sub_pose = message_filters.Subscriber(frame_topic, Frame)
        self.mf = message_filters.ApproximateTimeSynchronizer([sub_pose, sub_depth], 100, 0.5)
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
        
        #各部位の信頼度(point)
        file_path_point = '/home/kajilab/catkin_ws/src/ros_openpose/rosbag_file/rosbag_pointXYZ'
        file_name_point = f'Pub_point_{date_string}.bag' # ファイル名に日付を追加
        self.pointBag = rosbag.Bag(file_path_point + '/' + file_name_point,'w') # fileに書き込み
        
        self.all_degrees_R1 = np.array([])
        self.all_degrees_R2 = np.array([])
        self.all_degrees_L1 = np.array([])
        self.all_degrees_L2 = np.array([])
        self.save_point = np.array([])
        self.ez_array = np.array([])
        self.law_pass = np.array([])
        self.origin_deg = np.array([0,0])
        
        self.LPF_ez = np.array([0])
        self.x_stack = []
        
        self.RSholuder_array = np.array([])
        self.RElbow_array = np.array([])
        self.RWrist_array = np.array([])
        #self.all_degrees_L2 = np.array([])

        #self.all_degrees_R4 = np.array([])
        #self.all_degrees_L4 = np.array([])


    def ImageCallback(self, pose_data, depth_data):
        try:
            depth_image = self.bridge.imgmsg_to_cv2(depth_data, '16UC1')
            depth_image = cv2.resize(depth_image, (int(depth_image.shape[1]*0.25), int(depth_image.shape[0]*0.25)))
            #bodyPart.pointはFrame.msgから引っ張ってくる。(三次元座標をやりたかったらbodyPart.points?????)
            #person,bodypartは変数
            #pose_data.person:人数のリスト,person.bodyParts:座標情報のリスト(pointの情報)
            
            #point:2次元情報,point:3次元情報
            
            #text = [bodyPart.point for person in np.array(pose_data.persons) for bodyPart in np.array(person.bodyParts)]
            text = [bodyPart.point for person in np.array(pose_data.persons) for bodyPart in np.array(person.bodyParts)]
            score = [bodyPart.score for person in np.array(pose_data.persons) for bodyPart in np.array(person.bodyParts)]
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
                    error_x = abs(pose_data.persons[0].bodyParts[0].point.x - 0.05)
                    person_x = np.append(person_x,error_x)
                
                target = np.argmin(person_x)
                
                #ここで,奥行き2.0m以内に人がいた場合に処理するように制限を設けた.(首の座標)

                if pose_data.persons[target].bodyParts[1].point.z <= 2.5:
                
                #顔
                    point_REye = pose_data.persons[target].bodyParts[15].point
                    
                #上半身の座標
                    point_nose = pose_data.persons[target].bodyParts[0].point
                    point_Neck = pose_data.persons[target].bodyParts[1].point
                    point_RShoulder = pose_data.persons[target].bodyParts[2].point
                    point_RElbow = pose_data.persons[target].bodyParts[3].point
                    point_RWrist = pose_data.persons[target].bodyParts[4].point
                    point_LShoulder = pose_data.persons[target].bodyParts[5].point
                    point_LElbow = pose_data.persons[target].bodyParts[6].point 
                    point_LWrist = pose_data.persons[target].bodyParts[7].point
                    
                #上半身の信頼度
                    score_RSho =  pose_data.persons[target].bodyParts[2].score
                    score_REl =  pose_data.persons[target].bodyParts[3].score
                

                
            
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
            
            
            #横軸,縦軸,奥行きの順番
            
            robot_RSholuder = camera_to_robot_coordinates(point_RShoulder)
            #print("test",robot_RSholuder)
            #print("右肩 X: {} Y: {}, Z: {}".format(robot_RSholuder[0], robot_RSholuder[1], robot_RSholuder[2]))
            
            #右肘
            robot_RElbow = camera_to_robot_coordinates(point_RElbow)
            #print("右肘 X: {}, Y: {}, Z: {}".format(point_RElbow.x, point_RElbow.y ,point_RElbow.z))
            #print("右肘 X: {}, Y: {}, Z: {}".format(robot_RElbow[0], robot_RElbow[1] ,robot_RElbow[2]))
            
            
            
            #右手首
            robot_RWrist = camera_to_robot_coordinates(point_RWrist)
            #print("右手首 X: {}, Y: {}, Z: {}".format(robot_x4, robot_y4 ,robot_z4))
            
            #左肩
            robot_LSholuder = camera_to_robot_coordinates(point_LShoulder)
            
            #左肘
            robot_LElbow = camera_to_robot_coordinates(point_LElbow)
            
            #左手首
            robot_LWrist = camera_to_robot_coordinates(point_LWrist)
            
            self.RSholuder_array = np.append(self.RSholuder_array,robot_RSholuder)
            
            Not_NAN_RSholuder = replace_nan_with_previous_ez(self.RSholuder_array)
            
            self.RElbow_array = np.append(self.RElbow_array,robot_RElbow)
            
            Not_NAN_RElbow = replace_nan_with_previous_ez(self.RElbow_array)
            
            self.RWrist_array = np.append(self.RWrist_array,robot_RWrist)
            
            Not_NAN_RWrist = replace_nan_with_previous_ez(self.RWrist_array)
            
            
            #print("Not_NAN_-1",Not_NAN_RElbow[-1])
            
            #filter_RElbow = medfilt(Not_NAN_RElbow, kernel_size=15)
            
            #filter_RElbow = kalman_filter(self.RElbow_array)
            
            #temp = filter_RElbow[-1]
            
            #print("r_Elbow",temp)
            
            
            
            
            #角度摘出の部分
            ez = ChUnitVector(robot_RSholuder,robot_RElbow)
            
            print("ez",ez)
            
            self.ez_array = np.append(self.ez_array,ez)
            
            Not_NAN_ez = replace_nan_with_previous_ez(self.ez_array)
            
            print("Not_NAN",Not_NAN_ez)
            
            #print("pokemon",Not_NAN_ez)
            
            #self.law_pass = np.append(self.law_pass,Not_NAN_ez)
            """
            
            self.Med_RElbow = np.append(self.Med_RElbow,Not_NAN_RElbow[0])
            
            test = medfilt(self.Med_RElbow,15)
            
            robot_RElbow[0] = test[-1]
            
            alpha = 0.1
            self.LPF_ez = (1.0-alpha)*self.LPF_ez + alpha*Not_NAN_ez
            
            print("LPF_ez",self.LPF_ez)
            """
            
            # ガウシアンフィルタと指数移動平均を組み合わせて適用
            #self.LPF_ez = apply_median_filter(Not_NAN_ez)
            
            """
            self.x_stack.append(Not_NAN_ez)
            if len(self.x_stack) > 10:
                self.x_stack.pop(0)
                self.LPF_ez = np.median(self.x_stack,axis=0)
            """
            
            #print("LPF_ez",self.LPF_ez)
            
            """
            print("NotNAN_x",Not_NAN_ez[-3])
            print("NotNAN_y",Not_NAN_ez[-2])
            print("NotNAN_z",Not_NAN_ez[-1])
            """
            
            #qe1,qe2 = shoulder_ik(Not_NAN_ez)
            
            deg_R1,deg_R2 = shoulder_ik_R(Not_NAN_ez)

            print("Sholuder pitch",np.rad2deg(deg_R1))
            print("Sholuder roll",np.rad2deg(deg_R2))
            #print("q_suitei",qe)


            #print("r",r)
            #NAN埋めをしようとする配列の作成
            #self.all_degrees_L1 = np.append(self.all_degrees_L2,deg_L1)
            #self.all_degrees_L2 = np.append(self.all_degrees_L2,deg_L2)
            self.all_degrees_R1 = np.append(self.all_degrees_R1,deg_R1)
            self.all_degrees_R2 = np.append(self.all_degrees_R2,deg_R2)
            
            
            #self.all_degrees_R4 = np.append(self.all_degrees_R4,deg_R3)
            #self.all_degrees_L4 = np.append(self.all_degrees_L4,deg_L4)
            
            #print(self.all_degrees)
            
            #NAN埋めを処理している部分
            
            
            #Deg_Not_NAN_L1 = replace_nan_with_previous(self.all_degrees_L1)
            #Deg_Not_NAN_L2 = replace_nan_with_previous(self.all_degrees_L2)
            Deg_Not_NAN_R1 = replace_nan_with_previous(self.all_degrees_R1)
            Deg_Not_NAN_R2 = replace_nan_with_previous(self.all_degrees_R2)
            
            #Deg_Not_NAN_R4 = replace_nan_with_previous(self.all_degrees_R4)
            #Deg_Not_NAN_L4 = replace_nan_with_previous(self.all_degrees_L4)
            
            #Deg_EMA_R1 = calculate_ema(Deg_Not_NAN_R1,0.6)
            #Deg_EMA_R2 = calculate_ema(Deg_Not_NAN_R2,0.6)
            
            #NAN埋めした配列の末尾を取り出す部分
            
            Deg_R1 = Deg_Not_NAN_R1[-1]
            Deg_R2 = Deg_Not_NAN_R2[-1]
            #Deg_R1 = Deg_EMA_R1[-1]
            #Deg_R2 = Deg_EMA_R2[-1]
            
            #print("pitch",Deg_R1)
            #print("Roll",Deg_R2)
            
            self.origin_deg = np.array([Deg_R1,Deg_R2])
            #print("q_org",self.origin_deg)
            
            #Deg_R4 = Deg_Not_NAN_R4[-1]
            #Deg_L4 = Deg_Not_NAN_L4[-1]
            
            #DegのROSbagファイルの作成の部分
            #Deg = np.array([Deg_L1,Deg_L2,Deg_R1,Deg_R2])
            Deg = np.array([robot_RElbow[0],robot_RElbow[1],robot_RElbow[2]])
            
            #print("Light Roll",Deg_L1)
            #print("Light pitch",Deg_L2)
            #print("Right pitch",Deg_R1)
            #print("Right roll",Deg_R2)
            

            
            Pub_Deg = Float32MultiArray(data = Deg)
            
            
            
            #角度変換(limit制限)
            #conv_deg_L1 = limit_deg(Deg_L1,100,-95)
            #conv_deg_L2 = limit_deg(Deg_L2,135,0)
            conv_deg_R1 = limit_deg(Deg_R1,100,-95)
            conv_deg_R2 = limit_deg(Deg_R2,135,0)
            
            #conv_deg_R4 = limit_deg(Deg_R4,25.0,-125.0)
            #conv_deg_L4 = limit_deg(Deg_L4,25.0,-125.0)
            
            
            #角度変換した角度の配列の格納
            #conv = np.array([conv_deg_L1,conv_deg_L2,conv_deg_R1,conv_deg_R2])
            conv = np.array([conv_deg_R1,conv_deg_R2])
            Pub_conv = Float32MultiArray(data = conv)
            
            #各部位の信頼度(point)を格納
            
            point = np.array([Not_NAN_RSholuder[0],Not_NAN_RSholuder[1],Not_NAN_RSholuder[2],Not_NAN_RElbow[0],Not_NAN_RElbow[1],Not_NAN_RElbow[2],Not_NAN_RWrist[0],Not_NAN_RWrist[1],Not_NAN_RWrist[2]])
            Pub_point = Float32MultiArray(data = point)
            
            #Pub_conv値をパブリッシュ
            self.Pub_Conv_deg.publish(Pub_conv)
            
            #bagファイルにデータを書き込む(theta)
            self.DegBag.write('deg_array', Pub_Deg, rospy.Time.now())

             #bagファイルにデータを書き込む(conv_theta)
            self.convBag.write('conv_deg_array',Pub_conv,rospy.Time.now())
            
            #bagファイルにデータを書き込む(point)
            self.pointBag.write('point_array',Pub_point,rospy.Time.now())

        except:
            pass
    
    #bagファイルを閉じる処理
    def __del__(self):
            # ノードが終了するときにrosbagファイルを閉じる
        self.DegBag.close()
        self.convBag.close()
        self.pointBag.close()
        
if __name__ == '__main__':
    try:
        
        
        #ノードの宣言
        rospy.init_node('echo', anonymous=False)

        rate = rospy.Rate(20)  # 20 Hz

        while not rospy.is_shutdown():
                #10秒間起動を遅らせる
            rospy.sleep(10.0)
            rospy.loginfo("Start OpenPose")
            de = depth_estimater()

            rospy.sleep(30.0)
            
            rospy.loginfo("Stopping OpenPose")
            
            rospy.signal_shutdown("Program completed")
            
            rate.sleep()
        
    
        """
        # ノードの宣言
        rospy.init_node('echo', anonymous=False)
        
        # 10秒間待機
        rospy.sleep(5.0)
        
        # プログラムの開始
        rospy.loginfo("Start OpenPose")
        de = depth_estimater()
        
        # 20秒間実行
        rospy.sleep(30.0)
        
        # プログラムの停止
        rospy.loginfo("Stopping OpenPose")
        
        # すべての処理が終わったらノードをシャットダウン
        rospy.signal_shutdown("Program completed")
        """
        
    except rospy.ROSInterruptException:
        pass