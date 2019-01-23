#!/usr/bin/env python
#
#Copyright (c) 2018, Fredrik Johansson, Oskar Dahl
#All rights reserved.

import rospy
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from robot_msgs.msg import odometryData
import cv2
import cv_bridge
from detect_color import getGPSPos

myBridge = cv_bridge.CvBridge()

pub = rospy.Publisher("g5_gps_pos_channel", odometryData, queue_size=10)

odometryMsg = odometryData()

def callback(data):
	convertedImg = myBridge.imgmsg_to_cv2(data) # Convert image to cv2 format
	#resizedImg = cv2.resize(convertedImg, (640, 480)) #use this if you want to use a different sized image
	#cv2.imwrite('gps.jpg', convertedImg)

	X, Y, A = getGPSPos(convertedImg, 'blue')
	rospy.loginfo('X=%s, Y=%s, A=%s', X, Y, A)

	if X != 0 and Y != 0:
		odometryMsg.pos_x = X
		odometryMsg.pos_y = Y
		odometryMsg.pos_a = A
		odometryMsg.ros_time = rospy.rostime
		pub.publish(odometryMsg)

def gps():
	rospy.init_node('gps', anonymous=True)
	rospy.Subscriber('gps_channel', Image, callback)

	rate = rospy.Rate(10) # 5hz

	while not rospy.is_shutdown():
		rate.sleep()
		rospy.spin()


if __name__ == '__main__':
	try:
		gps()
	except rospy.ROSInterruptException:
		cv2.destroyAllWindows()
		pass
