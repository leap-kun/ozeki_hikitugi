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
        self.Pub_Conv_deg = rospy.Publisher('Pub_Conv_deg', Float32MultiArray, queue_size = 10,latch = True)
        self.Pub_point = rospy.Publisher('Pub_point',Float32MultiArray ,queue_size = 10 ,latch = True)
        
        #画像系のトピック
        
        self.bridge = CvBridge()
        frame_topic = rospy.get_param('~pub_topic')
        self.sub_pose = rospy.Subscriber(frame_topic, Frame, self.ImageCallback, queue_size=10)
        
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
        
        self.ez_array_R = np.array([])
        self.ez_array_L = np.array([])
        
        self.RSholuder_array = np.array([])
        self.RElbow_array = np.array([])
        self.RWrist_array = np.array([])

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
            
            #右手首
            robot_RWrist = camera_to_robot_coordinates(point_RWrist)
            #print("右手首 X: {}, Y: {}, Z: {}".format(robot_x4, robot_y4 ,robot_z4))
            
            #左肩
            robot_LSholuder = camera_to_robot_coordinates(point_LShoulder)
            
            #左肘
            robot_LElbow = camera_to_robot_coordinates(point_LElbow)
            
            
            self.RElbow_array = np.append(self.RElbow_array,robot_RElbow)
            
            Not_NAN_RElbow = replace_nan_with_previous_ez(self.RElbow_array)
            
            self.RWrist_array = np.append(self.RWrist_array,robot_RWrist)
            
            Not_NAN_RWrist = replace_nan_with_previous_ez(self.RWrist_array)

            
            #角度摘出の部分
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
            
            point = np.array([robot_RSholuder[0],robot_RSholuder[1],robot_RSholuder[2],Not_NAN_RElbow[0],Not_NAN_RElbow[1],Not_NAN_RElbow[2]])
            Pub_point = Float32MultiArray(data = point)
            
            #point = np.array([robot_LSholuder[0],robot_LSholuder[1],robot_LSholuder[2],robot_LElbow[0],robot_LElbow[1],robot_LElbow[2]])
            #Pub_point = Float32MultiArray(data = point)
            
            
            #Pub_conv値をパブリッシュ
            self.Pub_deg.publish(Pub_Deg)
            
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
            rospy.sleep(2.0)
            rospy.loginfo("Start OpenPose")
            de = depth_estimater()

            rospy.sleep(60.0)
            
            rospy.loginfo("Stopping OpenPose")
            
            rospy.signal_shutdown("Program completed")
            
            rate.sleep()
        
    except rospy.ROSInterruptException:
        pass