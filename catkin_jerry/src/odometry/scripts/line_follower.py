#!/usr/bin/env python

# Line follower

import rospy
import numpy as np
from robot_msgs.msg import sensorData
from robot_msgs.msg import ctrlData

LINETHRESHOLD = 850
I = 0
prevError = 0

PIDMsg = ctrlData()
pub = rospy.Publisher('g5_arduino_ctrl_channel', ctrlData, queue_size=10)

def calculatePID(Kp, Ki, Kd, err):
	global prevError
	global I
	P = err
	I += err
	D = err - prevError
	PIDval = (Kp*P) + (Ki*I) + (Kd*D)
	prevError = err
	return PIDval


def callback(data):
	global LINETHRESHOLD
	sensor_left = (data.IR_left > LINETHRESHOLD)
	sensor_center = (data.IR_center > LINETHRESHOLD)
	sensor_right = (data.IR_right > LINETHRESHOLD)

	# rospy.loginfo('IR_Left=%s, IR_Center=%s, IR_Right=%s', data.IR_left, data.IR_center, data.IR_right) #for debugging

	if sensor_left == 1 and sensor_center == 0 and sensor_right == 0:
		error = -2
	elif sensor_left == 1 and sensor_center == 1 and sensor_right == 0:
		error = -1	
	elif sensor_left == 0 and sensor_center == 1 and sensor_right == 0:
		error = 0
	elif sensor_left == 0 and sensor_center == 1 and sensor_right == 1:
		error = 1
	elif sensor_left == 0 and sensor_center == 0 and sensor_right == 1:
		error = 2
	else:
		error = 0

	currentPID = calculatePID(25, 0.01, 1, error)

	PIDMsg.speed_left = (70 + currentPID)
	PIDMsg.speed_right = (70 - currentPID)
	PIDMsg.ros_time = rospy.get_rostime()
	pub.publish(PIDMsg)

def control():
	rospy.init_node('line_follower', anonymous=True)
	rospy.Subscriber('g5_sensor_channel', sensorData, callback)

	rate = rospy.Rate(100) #100hz

	while not rospy.is_shutdown():
		rospy.spin()
		rate.sleep()


if __name__ == '__main__':
	try:
		control()
	except rospy.ROSInterruptException: 
		pass
