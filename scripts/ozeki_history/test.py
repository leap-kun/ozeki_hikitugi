#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class MovingObjectDetector:
    def __init__(self):
        rospy.init_node('moving_object_detector', anonymous=True)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber('/camera/image_raw', Image, self.image_callback)

        self.background = None
        self.alpha = 0.5  # 背景更新の重み

    def image_callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, 'bgr8')
        except CvBridgeError as e:
            print(e)
            return

        if self.background is None:
            # 最初のフレームを背景として設定
            self.background = cv_image.copy().astype(float)
            return

        # 背景差分法を使用して動く物体を検出
        diff = cv2.absdiff(self.background.astype(np.uint8), cv_image)
        gray_diff = cv2.cvtColor(diff, cv2.COLOR_BGR2GRAY)
        _, thresh = cv2.threshold(gray_diff, 30, 255, cv2.THRESH_BINARY)
        contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # 検出された物体を長方形で囲む
        for contour in contours:
            x, y, w, h = cv2.boundingRect(contour)
            cv2.rectangle(cv_image, (x, y), (x + w, y + h), (0, 255, 0), 2)

        # 背景を更新
        cv2.accumulateWeighted(cv_image, self.background, self.alpha)

        cv2.imshow('Moving Object Detection', cv_image)
        cv2.waitKey(1)

if __name__ == '__main__':
    try:
        detector = MovingObjectDetector()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
