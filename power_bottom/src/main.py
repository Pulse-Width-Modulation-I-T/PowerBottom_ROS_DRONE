#!/usr/bin/env python

import rospy
import cv2
from std_msgs.msg import String

rospy.init_node('power_bottom')

pub = rospy.Publisher('/test', String, queue_size=1)

r = rospy.Rate(5)

while not rospy.is_shutdown():
	pub.publish('Test')
	r.sleep()
