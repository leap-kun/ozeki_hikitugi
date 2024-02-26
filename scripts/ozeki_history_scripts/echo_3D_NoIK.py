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
from My_function_3D_NoIK import limit_deg
#atan2で角度計算
from My_function_3D_NoIK import atan2_deg
#カメラ画像からロボット座標に変換
from My_function_3D_NoIK import camera_to_robot_coordinates
#余弦定理を用いて,角度計算
#from My_function_3D import cosine_xz
#余弦定理を用いて,角度計算
from My_function_3D_NoIK import cosine_xyz
#NANが出た場合に一つ前の値を参照にして値を書き換える関数
from My_function_3D_NoIK import replace_nan_with_previous
#長方形(バウンディングボックスモドキ)の生成
#from My_function_3D import draw_rectangle
#角度の制限値
from My_function_3D_NoIK import limit_deg
from My_function_3D_NoIK import CameratoPixel



class depth_estimater:
    queue_size = 100
    fps = 10.
    delay = 0.5
    

    def __init__(self):
        
        #Use Myself RosTopic
        self.Pub_deg = rospy.Publisher('Pub_deg' , Float32MultiArray , queue_size = 10, latch = True)
        self.Pub_Conv_deg = rospy.Publisher('Pub_Conv_deg', Float32MultiArray, queue_size = 1,latch = True)
        self.Pub_Score = rospy.Publisher('Pub_Score',Float32MultiArray ,queue_size = 10 ,latch = True)
        
        #画像系のトピック
        
        self.bridge = CvBridge()
        frame_topic = rospy.get_param('~pub_topic')
        #sub_rgb = message_filters.Subscriber("/usb_cam/image_raw", Image)
        sub_depth =  message_filters.Subscriber("/depth_to_rgb/image_raw",Image)
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
        
        #各部位の信頼度(Score)
        file_path_score = '/home/kajilab/catkin_ws/src/ros_openpose/rosbag_file/rosbag_score'
        file_name_score = f'Pub_score_{date_string}.bag' # ファイル名に日付を追加
        self.scoreBag = rosbag.Bag(file_path_score + '/' + file_name_score,'w') # fileに書き込み
        
        self.all_degrees_R2 = np.array([])
        self.all_degrees_L2 = np.array([])

        self.all_degrees_R4 = np.array([])
        self.all_degrees_L4 = np.array([])


    def ImageCallback(self, pose_data, depth_data):
        try:
            depth_image = self.bridge.imgmsg_to_cv2(depth_data, '32FC1')
            depth_image = cv2.resize(depth_image, (int(depth_image.shape[1]*0.25), int(depth_image.shape[0]*0.25)))
            #bodyPart.pointはFrame.msgから引っ張ってくる。(三次元座標をやりたかったらbodyPart.points?????)
            #person,bodypartは変数
            #pose_data.person:人数のリスト,person.bodyParts:座標情報のリスト(pointの情報)
            
            #point:2次元情報,point:3次元情報
            
            #text = [bodyPart.point for person in np.array(pose_data.persons) for bodyPart in np.array(person.bodyParts)]
            text = [bodyPart.point for person in np.array(pose_data.persons) for bodyPart in np.array(person.bodyParts)]
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
                    error_x = abs(pose_data.persons[n].bodyParts[0].point.x - 640)
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
            robot_y1, robot_z1, robot_x1 = camera_to_robot_coordinates(point_Neck)
            #print("変換前:首 X: {}, Y: {}, Z: {}".format(point_Neck.x ,point_Neck.y, point_Neck.z))
            #print("首 X: {}, Y: {}, Z: {}".format(robot_x1,robot_y1 ,robot_z1))
            
            #右肩
            robot_y2, robot_z2, robot_x2 = camera_to_robot_coordinates(point_RShoulder)
            #print("右肩 X: {} Y: {}, Z: {}".format(robot_x2, robot_y2, robot_z2))
            
            #右肘
            robot_y3, robot_z3, robot_x3 = camera_to_robot_coordinates(point_RElbow)
            #print("右肘 X: {}, Y: {}, Z: {}".format(robot_x3, robot_y3 ,robot_z3))
            
            #右手首
            robot_y4, robot_z4, robot_x4 = camera_to_robot_coordinates(point_RWrist)
            #print("右手首 X: {}, Y: {}, Z: {}".format(robot_x4, robot_y4 ,robot_z4))
            
            #左肩
            robot_y5,robot_z5,robot_x5 = camera_to_robot_coordinates(point_LShoulder)
            
            #左肘
            robot_y6,robot_z6,robot_x6 = camera_to_robot_coordinates(point_LElbow)
            
            #左手首
            robot_y7,robot_z7,robot_x7 = camera_to_robot_coordinates(point_LWrist)
            
            #deg_R1 = atan2_deg(robot_x0,robot_x1,robot_x2,robot_y0,robot_y1,robot_y2)

            #deg_R2 = atan2_deg(robot_y0,robot_y1,robot_y2,robot_z0,robot_z1,robot_z2)
            
            #deg_R4_2D = cosine_xz(robot_x3,robot_x2,robot_x4,robot_z3,robot_z2,robot_z4)
            
            #角度摘出の部分

            deg_R2 = atan2_deg(robot_y2,robot_y1,robot_y3,robot_z2,robot_z1,robot_z3) - 90
            deg_L2 = atan2_deg(robot_y5,robot_y6,robot_y1,robot_z5,robot_z6,robot_z1) - 90
            """
            deg_R2 = atan2_deg(robot_u0,robot_u1,robot_u2,robot_v0,robot_v1,robot_v2)
            deg_L2 = atan2_deg(robot_u5,robot_u6,robot_u1,robot_v5,robot_v6,robot_v1)
            """
            deg_R4 = cosine_xyz(robot_x3,robot_x2,robot_x4,robot_y3,robot_y2,robot_y4,robot_z3,robot_z2,robot_z4,-1)
            deg_L4 = cosine_xyz(robot_x6,robot_x5,robot_x7,robot_y6,robot_y5,robot_y7,robot_z6,robot_z5,robot_z7,-1)
            
            
            
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
            
            print("L",Deg_L2)
            print("R",Deg_R2)
            
            #DegのROSbagファイルの作成の部分
            Deg = np.array([Deg_L2,Deg_L4,Deg_R2,Deg_R4])
            
            Pub_Deg = Float32MultiArray(data = Deg)
            
            
            
            #角度変換(limit制限)
            conv_deg_L2 = limit_deg(Deg_L2,135,0)
            conv_deg_R2 = limit_deg(Deg_R2,135,0)
            
            conv_deg_R4 = limit_deg(Deg_R4,25.0,-125.0)
            conv_deg_L4 = limit_deg(Deg_L4,25.0,-125.0)
            
            
            print("L",conv_deg_L2)
            print("R",conv_deg_R2)
            
            
            #角度変換した角度の配列の格納
            conv = np.array([conv_deg_L2,conv_deg_L4,conv_deg_R2,conv_deg_R4])
            Pub_conv = Float32MultiArray(data = conv)
            
            
            #各部位の信頼度(score)を格納
            
            score = get_score
            Pub_score = Float32MultiArray(data = score)
            
            #Pub_conv値をパブリッシュ
            self.Pub_Conv_deg.publish(Pub_conv)
            
            #bagファイルにデータを書き込む(theta)
            self.DegBag.write('deg_array', Pub_Deg, rospy.Time.now())

             #bagファイルにデータを書き込む(conv_theta)
            self.convBag.write('conv_deg_array',Pub_conv,rospy.Time.now())
            
            #bagファイルにデータを書き込む(score)
            self.scoreBag.write('score_array',Pub_score,rospy.Time.now())
            

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
        rospy.sleep(31.0)
        
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