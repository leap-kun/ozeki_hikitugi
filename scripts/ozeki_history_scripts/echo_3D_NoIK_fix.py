#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
from ros_openpose.msg import Frame

#from ros_opsepose.msg import Persons
import cv2
import sys
import message_filters
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import numpy as np
from std_msgs.msg import Float32MultiArray
import rosbag
from datetime import datetime

#以下がimportFile

#publishする角度の制限

#atan2で角度計算
from My_function_3D_NoIK_fix import atan2_deg
#カメラ画像からロボット座標に変換
from My_function_3D_NoIK_fix import camera_to_robot_coordinates
#余弦定理を用いて,角度計算
#from My_function_3D import cosine_xz
#余弦定理を用いて,角度計算
from My_function_3D_NoIK_fix import cosine_xyz
#NANが出た場合に一つ前の値を参照にして値を書き換える関数
from My_function_3D_NoIK_fix import replace_nan_with_previous
from My_function_3D_NoIK_fix import replace_nan_with_previous_ez



class depth_estimater:
    queue_size = 100
    fps = 10.
    delay = 0.5
    

    def __init__(self):
        
        #Use Myself RosTopic
        self.Pub_deg = rospy.Publisher('Pub_deg' , Float32MultiArray , queue_size = 10, latch = True)
        self.Pub_Conv_deg = rospy.Publisher('Pub_Conv_deg', Float32MultiArray, queue_size = 1,latch = True)
        self.Pub_point_L = rospy.Publisher('Pub_point_L',Float32MultiArray ,queue_size = 10 ,latch = True)
        self.Pub_point_R = rospy.Publisher('Pub_point_R',Float32MultiArray ,queue_size = 10 ,latch = True)
        
        #画像系のトピック
        
        self.bridge = CvBridge()
        frame_topic = rospy.get_param('~pub_topic')
        self.sub_pose = rospy.Subscriber(frame_topic, Frame, self.ImageCallback, queue_size=10)
        #rosbagファイルの作成
        
        now = datetime.now()#現在の日時を習得
        date_string = now.strftime("%Y_%m_%d_%H:%M")  # 日付を指定の形式に変換

        #thetaのpath
        file_path_theta = '/home/kajilab/catkin_ws/src/ros_openpose/rosbag_file/NoIK/rosbag_deg'
        file_name_theta = f'NoIK_Pub_deg_{date_string}.bag' # ファイル名に日付を追加
        self.DegBag = rosbag.Bag(file_path_theta + '/' + file_name_theta,'w') # fileに書き込み
        
        #theta変更後の角度のpath
        file_path_conv = '/home/kajilab/catkin_ws/src/ros_openpose/rosbag_file/NoIK/rosbag_conv_theta'
        file_name_conv = f'NoIK_Pub_Conv_theta_{date_string}.bag' # ファイル名に日付を追加
        self.convBag = rosbag.Bag(file_path_conv + '/' + file_name_conv,'w') # fileに書き込み
        
        #各部位の右肘(point)
        file_path_point_L = '/home/kajilab/catkin_ws/src/ros_openpose/rosbag_file/NoIK/point_L'
        file_name_point_L = f'NoIK_Pub_point_{date_string}.bag' # ファイル名に日付を追加
        self.pointLBag = rosbag.Bag(file_path_point_L + '/' + file_name_point_L,'w') # fileに書き込み
        
        #各部位の信頼度(point)
        file_path_point_R = '/home/kajilab/catkin_ws/src/ros_openpose/rosbag_file/NoIK/point_R'
        file_name_point_R = f'NoIK_sPub_point_{date_string}.bag' # ファイル名に日付を追加
        self.pointRBag = rosbag.Bag(file_path_point_R + '/' + file_name_point_R,'w') # fileに書き込み
        
        
        
        self.all_degrees_R2 = np.array([])
        self.all_degrees_L2 = np.array([])

        self.all_degrees_R4 = np.array([])
        self.all_degrees_L4 = np.array([])


    def ImageCallback(self, pose_data):
        try:
            #bodyPart.pointはFrame.msgから引っ張ってくる。(三次元座標をやりたかったらbodyPart.points?????)
            #person,bodypartは変数
            #pose_data.person:人数のリスト,person.bodyParts:座標情報のリスト(pointの情報)
            
            #point:2次元情報,point:3次元情報
            
            #text = [bodyPart.point for person in np.array(pose_data.persons) for bodyPart in np.array(person.bodyParts)]
            text = [bodyPart.point for person in np.array(pose_data.persons) for bodyPart in np.array(person.bodyParts)]
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
                    error_x = abs(pose_data.persons[n].bodyParts[0].point.x - 0.05)
                    person_x = np.append(person_x,error_x)
                
                target = np.argmin(person_x)
                
                #ここで,奥行き2.0m以内に人がいた場合に処理するように制限を設けた.(首の座標)

                if pose_data.persons[target].bodyParts[1].point.z <= 2.0:
                
                #顔
                    point_REye = pose_data.persons[target].bodyParts[15].point
                
                #上半身
                    point_Neck = pose_data.persons[target].bodyParts[1].point
                    point_RShoulder = pose_data.persons[target].bodyParts[2].point
                    point_RElbow = pose_data.persons[target].bodyParts[3].point
                    point_RWrist = pose_data.persons[target].bodyParts[4].point
                    point_LShoulder = pose_data.persons[target].bodyParts[5].point
                    point_LElbow = pose_data.persons[target].bodyParts[6].point 
                    point_LWrist = pose_data.persons[target].bodyParts[7].point
                
                #print(point_Neck.x)
                
                    """
                    #下半身
                
                    point_MidHip = pose_data.persons[target].bodyParts[8].point 
                    point_RHip = pose_data.persons[target].bodyParts[9].point
                    point_RKnee = pose_data.persons[target].bodyParts[10].point 
                
                    point_LHip = pose_data.persons[target].bodyParts[12].point 
                    point_LKnee = pose_data.persons[target].bodyParts[13].point 
    
                    """
                #point_LBigToe = pose_data.persons[target].bodyParts[19].point
                
            
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
            
            #首
            robot_Neck = camera_to_robot_coordinates(point_Neck)
            #print("変換前:首 X: {}, Y: {}, Z: {}".format(point_Neck.x ,point_Neck.y, point_Neck.z))
            #print("首 X: {}, Y: {}, Z: {}".format(robot_x1,robot_y1 ,robot_z1))
            
            #右肩
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
            #角度摘出の部分

            deg_R2 = atan2_deg(robot_RSholuder,robot_RElbow,robot_Neck) - 90
            deg_L2 = atan2_deg(robot_LSholuder,robot_Neck,robot_LElbow) - 90
            """
            deg_R2 = atan2_deg(robot_u0,robot_u1,robot_u2,robot_v0,robot_v1,robot_v2)
            deg_L2 = atan2_deg(robot_u5,robot_u6,robot_u1,robot_v5,robot_v6,robot_v1)
            """
            deg_R4 = cosine_xyz(robot_RElbow,robot_RWrist,robot_RSholuder,-1)
            deg_L4 = cosine_xyz(robot_LElbow,robot_LWrist,robot_LSholuder,-1)
            
            
            #NAN埋めをしようとする配列の作成
            self.all_degrees_R2 = np.append(self.all_degrees_R2,deg_R2)
            self.all_degrees_L2 = np.append(self.all_degrees_L2,deg_L2)
            
            self.all_degrees_R4 = np.append(self.all_degrees_R4,deg_R4)
            self.all_degrees_L4 = np.append(self.all_degrees_L4,deg_L4)
            
            #print(self.all_degrees)
            
            #NAN埋めを処理している部分
            
            Deg_Not_NAN_R2 = replace_nan_with_previous(self.all_degrees_R2)
            Deg_Not_NAN_L2 = replace_nan_with_previous(self.all_degrees_L2)
            
            
            Deg_Not_NAN_R4 = replace_nan_with_previous(self.all_degrees_R4)
            Deg_Not_NAN_L4 = replace_nan_with_previous(self.all_degrees_L4)
            
            #NAN埋めした配列の末尾を取り出す部分
            Deg_R2 = Deg_Not_NAN_R2[-1]
            Deg_L2 = Deg_Not_NAN_L2[-1]
            Deg_R4 = Deg_Not_NAN_R4[-1]
            Deg_L4 = Deg_Not_NAN_L4[-1]
            
            print("roll L",Deg_L2)
            print("roll R",Deg_R2)
            
            print("hiji L",Deg_L4)
            print("hiji R",Deg_R4)
            
            
            #DegのROSbagファイルの作成の部分
            Deg = np.array([Deg_L2,Deg_L4,Deg_R2,Deg_R4])
            
            Pub_Deg = Float32MultiArray(data = Deg)
            
            
            
            #角度変換(limit制限)
            conv_deg_L2 = np.clip(Deg_L2,0,135)
            conv_deg_R2 = np.clip(Deg_R2,0,135)
            
            conv_deg_R4 = np.clip(Deg_R4,-125.0,25.0)
            conv_deg_L4 = np.clip(Deg_L4,-125.0,25.0)
            
            
            print("L",conv_deg_L2)
            print("R",conv_deg_R2)
            
            
            #角度変換した角度の配列の格納
            conv = np.array([conv_deg_L2,conv_deg_L4,conv_deg_R2,conv_deg_R4])
            Pub_conv = Float32MultiArray(data = conv)
            
            
            point_L = np.array([robot_LSholuder[1],robot_LSholuder[2],robot_LElbow[1],robot_LElbow[2],robot_LWrist[1],robot_LWrist[2]])
            Pub_point_L = Float32MultiArray(data = point_L)
            
            point_R = np.array([robot_RSholuder[1],robot_RSholuder[2],robot_RElbow[1],robot_RElbow[2],robot_LWrist[1],robot_LWrist[2]])
            Pub_point_R = Float32MultiArray(data = point_R)
            
            
            #Pub_conv値をパブリッシュ
            self.Pub_Conv_deg.publish(Pub_conv)
            
            #bagファイルにデータを書き込む(theta)
            self.DegBag.write('deg_array', Pub_Deg, rospy.Time.now())

             #bagファイルにデータを書き込む(conv_theta)
            self.convBag.write('conv_deg_array',Pub_conv,rospy.Time.now())
            
            #bagファイルにデータを書き込む(point)
            self.pointLBag.write('point_array_L',Pub_point_L,rospy.Time.now())
            
            #bagファイルにデータを書き込む(point)
            self.pointRBag.write('point_array_R',Pub_point_R,rospy.Time.now())


        except:
            pass
    
    #bagファイルを閉じる処理
    def __del__(self):
            # ノードが終了するときにrosbagファイルを閉じる
        self.DegBag.close()
        self.convBag.close()
        self.pointLBag.close()
        self.pointRBag.close()
        
if __name__ == '__main__':
    try:
        
        """
        #ノードの宣言
        rospy.init_node('echo', anonymous=False)
        #10秒間起動を遅らせる
        rospy.sleep(5.0)
        rospy.loginfo("Start OpenPose")
        de = depth_estimater()
        rospy.spin()
        """
        
        # ノードの宣言
        rospy.init_node('echo', anonymous=False)
        
        # 10秒間待機
        rospy.sleep(10.0)
        
        # プログラムの開始
        rospy.loginfo("Start OpenPose")
        de = depth_estimater()
        
        # 20秒間実行
        rospy.sleep(30.0)
        
        # プログラムの停止
        rospy.loginfo("Stopping OpenPose")
        
        # すべての処理が終わったらノードをシャットダウン
        rospy.signal_shutdown("Program completed")

        
    except rospy.ROSInterruptException:
        pass