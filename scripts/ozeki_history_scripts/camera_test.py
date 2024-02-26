#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2

class CameraSubscriber:
    def __init__(self):
        # ROSノードの初期化
        rospy.init_node('camera_subscriber', anonymous=True)

        # カメラの映像を受け取るためのSubscriberを設定
        self.image_sub = rospy.Subscriber("/usb_cam/image_raw", Image, self.callback)

        # OpenCVとROSの画像変換のためのブリッジ
        self.bridge = CvBridge()

    def callback(self, data):
        try:
            # ROSイメージメッセージをOpenCVイメージに変換
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
            return

        # 長方形の座標と色を設定
        start_point = (100, 100)  # 長方形の左上の座標 (x, y)
        end_point = (300, 200)    # 長方形の右下の座標 (x, y)
        color = (0, 255, 0)       # 長方形の色 (B, G, R)
        thickness = 2             # 長方形の線の太さ

        # 長方形を描画
        cv_image_with_rectangle = cv2.rectangle(cv_image, start_point, end_point, color, thickness)

        # 描画したフレームを表示（ここでは表示せず、他の処理を追加することも可能）
        cv2.imshow('Frame with Rectangle', cv_image_with_rectangle)
        cv2.waitKey(1)

def main():
    camera_subscriber = CameraSubscriber()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
