#!/usr/bin/env python
#
#Copyright (c) 2017, Martin Cooney
#All rights reserved.
#THIS SOFTWARE IS PROVIDED "AS IS".
#IN NO EVENT WILL THE COPYRIGHT OWNER BE LIABLE FOR ANY DAMAGES
#ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE.

import rospy
import numpy as np
from std_msgs.msg import String
from subprocess import call
import sys
from sensor_msgs.msg import Image
import cv2
import cv_bridge

originalCameraMatrix=np.array([[540.92809281, 0, 310.49944426], [0, 540.23283561, 227.12758261], [0, 0, 1]],np.float64)
optimalCameraMatrix=np.array([[457.13543701, 0, 306.81114822], [0, 493.90228271, 223.85634775], [0, 0, 1]],np.float64)
distortionCoefficients=np.array([[-4.01495171e-01, 1.93524711e-01, -1.71371113e-03, -1.11895861e-04, -3.48628258e-02]],np.float64)

def gps():
	global originalCameraMatrix
	global optimalCameraMatrix
	global distortionCoefficients

	rospy.init_node('gps', anonymous=True)
	image_pub= rospy.Publisher("g5_gps_channel", Image, queue_size=10)
	rate=rospy.Rate(10)
	myBridge= cv_bridge.CvBridge()
	cameraGetImageCommand = "wget --user root --password r3dM1lk -q -O current.jpg http://192.168.1.2/axis-cgi/jpg/image.cgi"
	while not rospy.is_shutdown():
		# rospy.loginfo("published image") #for debugging
		call(cameraGetImageCommand, shell=True)
		img = cv2.imread("current.jpg")
		calibratedImg = cv2.undistort(img, originalCameraMatrix, distortionCoefficients, None, optimalCameraMatrix)
		cv2.imwrite('calibrated.jpg', calibratedImg)
		
		# resized_img = cv2.resize(img, (640, 480)) #use this if you want to use a different sized image
		convertedImg = myBridge.cv2_to_imgmsg(calibratedImg, encoding="bgr8")
		image_pub.publish(convertedImg)
		rate.sleep()
	
	cv2.destroyAllWindows()

if __name__== '__main__':
	try:
		gps()
	except rospy.ROSInterruptException:
		cv2.destroyAllWindows()
		pass
