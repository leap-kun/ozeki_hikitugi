#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
from ros_openpose.msg import Frame

#from ros_opsepose.msg import Persons

from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from std_msgs.msg import Float32MultiArray
import rosbag
from datetime import datetime

#以下がimportFile

#publishする角度の制限
from My_function_3D import camera_to_robot_coordinates
from My_function_3D import replace_nan_with_previous_ez
from My_function_3D import ChUnitVector
from My_function_3D import shoulder_ik_R
from My_function_3D import shoulder_ik_L

class depth_estimater:
    queue_size = 100
    fps = 10.
    delay = 0.5
    

    def __init__(self):
        
        #Use Myself RosTopic
        self.Pub_deg = rospy.Publisher('Pub_deg' , Float32MultiArray , queue_size = 1, latch = True)
        self.Pub_point_L = rospy.Publisher('Pub_point_L',Float32MultiArray ,queue_size = 10 ,latch = True)
        self.Pub_point_R = rospy.Publisher('Pub_point_R',Float32MultiArray ,queue_size = 10 ,latch = True)
        self.Pub_deg_R = rospy.Publisher('Pub_deg_R', Float32MultiArray, queue_size = 10,latch = True)
        self.Pub_deg_L = rospy.Publisher('Pub_deg_L', Float32MultiArray, queue_size = 10,latch = True)
        
        #画像系のトピック
        
        self.bridge = CvBridge()
        frame_topic = rospy.get_param('~pub_topic')
        self.sub_pose = rospy.Subscriber(frame_topic, Frame, self.ImageCallback, queue_size=10)
        
        #rosbagファイルの作成
        
        now = datetime.now()#現在の日時を習得
        date_string = now.strftime("%Y_%m_%d_%H:%M")  # 日付を指定の形式に変換

        #各部位の右肘(point)
        file_path_point_L = '/home/kajilab/catkin_ws/src/ros_openpose/rosbag_file/IK/point_L'
        file_name_point_L = f'IK_Pub_point_L_{date_string}.bag' # ファイル名に日付を追加
        self.pointLBag = rosbag.Bag(file_path_point_L + '/' + file_name_point_L,'w') # fileに書き込み
        
        #各部位の信頼度(point)
        file_path_point_R = '/home/kajilab/catkin_ws/src/ros_openpose/rosbag_file/IK/point_R'
        file_name_point_R = f'IK_Pub_point_R_{date_string}.bag' # ファイル名に日付を追加
        self.pointRBag = rosbag.Bag(file_path_point_R + '/' + file_name_point_R,'w') # fileに書き込み
        
        #右肘
        file_path_degR = '/home/kajilab/catkin_ws/src/ros_openpose/rosbag_file/IK/deg_R'
        file_name_degR = f'IK_Pub_degR_{date_string}.bag' # ファイル名に日付を追加
        self.degR = rosbag.Bag(file_path_degR + '/' + file_name_degR,'w') # fileに書き込み
        
        #theta変更後の角度のpath
        file_path_degL = '/home/kajilab/catkin_ws/src/ros_openpose/rosbag_file/IK/deg_L'
        file_name_degL = f'IK_Pub_degL_{date_string}.bag' # ファイル名に日付を追加
        self.degL = rosbag.Bag(file_path_degL + '/' + file_name_degL,'w') # fileに書き込み
        
        self.ez_array_R = np.array([])
        self.ez_array_L = np.array([])
    
        self.all_degrees_R4 = np.array([])
        self.all_degrees_L4 = np.array([])
        
        self.RElbow_array = np.array([])
        self.LElbow_array = np.array([])


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
            
            #左肩
            robot_LSholuder = camera_to_robot_coordinates(point_LShoulder)
            
            #左肘
            robot_LElbow = camera_to_robot_coordinates(point_LElbow)
            
            
            self.RElbow_array = np.append(self.RElbow_array,robot_RElbow)
            
            Not_NAN_RElbow = replace_nan_with_previous_ez(self.RElbow_array)
            
            self.LElbow_array = np.append(self.LElbow_array,robot_LElbow)
            
            Not_NAN_LElbow = replace_nan_with_previous_ez(self.LElbow_array)

            
            #単位ベクトル摘出の部分
            ez_R = ChUnitVector(robot_RSholuder,robot_RElbow)
            ez_L = ChUnitVector(robot_LSholuder,robot_LElbow)

            self.ez_array_R = np.append(self.ez_array_R,ez_R)
            
            Not_NAN_ez_R = replace_nan_with_previous_ez(self.ez_array_R)
            
            self.ez_array_L = np.append(self.ez_array_L,ez_L)
            
            Not_NAN_ez_L = replace_nan_with_previous_ez(self.ez_array_L)
            
            #print("Not_NAN",Not_NAN_ez)
            
            #qe1,qe2 = shoulder_ik(Not_NAN_ez)
            
            deg_R1,deg_R2 = shoulder_ik_R(Not_NAN_ez_R)
            deg_L1,deg_L2 = shoulder_ik_L(Not_NAN_ez_L)
            
            
            print("Sholuder pitch",deg_L1)
            print("Sholuder roll",deg_L2)

            Deg = np.array([deg_L1,deg_L2,deg_R1,-deg_R2])
            #Deg = np.array([deg_R1,-deg_R2])
            Pub_Deg = Float32MultiArray(data = Deg)
            
            #print("Pub_deg",Pub_Deg)
            
            
            #角度変換した角度の配列の格納
            #conv = np.array([conv_deg_L1,conv_deg_L2,conv_deg_R1,conv_deg_R2])
            conv = np.array([0,0])
            Pub_conv = Float32MultiArray(data = conv)
            
            #各部位の信頼度(point)を格納
            
            point_L = np.array([robot_LSholuder[0],robot_LSholuder[1],robot_LSholuder[2],Not_NAN_LElbow[0],Not_NAN_LElbow[1],Not_NAN_LElbow[2]])
            Pub_point_L = Float32MultiArray(data = point_L)
            
            point_R = np.array([robot_RSholuder[0],robot_RSholuder[1],robot_RSholuder[2],Not_NAN_RElbow[0],Not_NAN_RElbow[1],Not_NAN_RElbow[2]])
            Pub_point_R = Float32MultiArray(data = point_R)
            
            deg_R = np.array([deg_R1,deg_R2])
            Pub_deg_R = Float32MultiArray(data = deg_R)
            
            deg_L = np.array([deg_L1,deg_L2])
            Pub_deg_L = Float32MultiArray(data = deg_L)
            
            
            #Pub_conv値をパブリッシュ
            self.Pub_deg.publish(Pub_Deg)
            
            #bagファイルにデータを書き込む(point)
            self.pointLBag.write('point_array_L',Pub_point_L,rospy.Time.now())
            
            #bagファイルにデータを書き込む(point)
            self.pointRBag.write('point_array_R',Pub_point_R,rospy.Time.now())
            
            #bagファイルにデータを書き込む(conv_theta)
            self.degL.write('Pub_deg_L',Pub_deg_L,rospy.Time.now())
            
            self.degR.write('Pub_deg_R',Pub_deg_R,rospy.Time.now())
            
        except:
            pass
    
    #bagファイルを閉じる処理
    def __del__(self):
            # ノードが終了するときにrosbagファイルを閉じる
        self.pointLBag.close()
        self.pointRBag.close()
        self.degL.close()
        self.degR.close()
        
if __name__ == '__main__':
    try:
        #ノードの宣言
        rospy.init_node('echo', anonymous=False)

        rate = rospy.Rate(20)  # 20 Hz

        while not rospy.is_shutdown():
                #10秒間起動を遅らせる
            rospy.sleep(31.0)
            rospy.loginfo("Start OpenPose")
            de = depth_estimater()

            rospy.sleep(30.0)
            
            rospy.loginfo("Stopping OpenPose")
            
            rospy.signal_shutdown("Program completed")
            
            rate.sleep()
        
    except rospy.ROSInterruptException:
        pass