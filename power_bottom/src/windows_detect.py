#!/usr/bin/env python3
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from clover import long_callback
from power_bottom import set_home_cb

class ImageNode:
    def __init__(self):
        rospy.init_node('windows_detect')

        self.bridge = CvBridge()

        self.pub = rospy.Publisher('~debug', Image, queue_size=1)

        rospy.Subscriber('main_camera/image_raw', Image, self.callback)

        rospy.loginfo("windows_detect started")

    @long_callback
    def callback(self, msg):
        img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

        h, w = img.shape[:2]
        cv2.circle(img, (w//2, h//2), 40, (0,255,0), 3)

        out_msg = self.bridge.cv2_to_imgmsg(img, encoding='bgr8')
        self.pub.publish(out_msg)
        

node = ImageNode()
rospy.spin()
