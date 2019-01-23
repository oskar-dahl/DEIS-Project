#!/usr/bin/env python

import rospy
#import numpy as np
from robot_msgs.msg import sensorData
from robot_msgs.msg import odometryData
import math

odometryMsg = odometryData()
pub = rospy.Publisher('g5_pos_channel', odometryData, queue_size=1)

# Robot
wheelBase = 158
wheelDiameter = 65
wheelCircumference = wheelDiameter * math.pi
countsPerRev = 192 # 4 pairs of N-S x 48:1 gearbox = 192 ticks per wheel rev
mmPerPulse = (math.pi*wheelDiameter)/countsPerRev # [mm / pulse]


# Position and encoder variables
initPos				 	= [20, 20, 0] # Init pos
position				= [initPos[0], initPos[1], initPos[2]] # X, Y, A
positionPrev			= [initPos[0], initPos[1], initPos[2]] # X, Y, A
distanceTraveled		= [0, 0] # right, left
distanceTraveledPrev	= [0, 0]  # right, left

# Flags
firstTime = 1

# =================================

def deadReckoning(rTicks=0, lTicks=0):

	global position
	global positionPrev
	global distanceTraveled
	global distanceTraveledPrev
	global firstTime
	global wheelBase
	global wheelCircumference
	global countsPerRev

	# Convert encoder ticks to distance
	distanceTraveled[0] = ((rTicks*mmPerPulse)/360)*(math.pi*wheelDiameter)
	distanceTraveled[1] = ((lTicks*mmPerPulse)/360)*(math.pi*wheelDiameter)

	#distanceTraveled[0] = wheelCircumference * rTicks / countsPerRev
	##distanceTraveled[1] = wheelCircumference * lTicks / countsPerRev

	#print("distanceTraveledRight = " + str(distanceTraveled[0]) + " distanceTraveledLeft = " + str(distanceTraveled[1]))

	if firstTime == 1:
		distanceTraveledPrev[0] = distanceTraveled[0]
		distanceTraveledPrev[1] = distanceTraveled[1]
		firstTime = 0

	# Start calc odometry
	deltaDistanceRight = distanceTraveled[0] - distanceTraveledPrev[0]
	deltaDistanceLeft  = distanceTraveled[1] - distanceTraveledPrev[1]

	#print("deltaDistanceRight = " + str(deltaDistanceRight) + " deltaDistanceLeft = " + str(deltaDistanceLeft))

	deltaDistance = (deltaDistanceRight + deltaDistanceLeft) / 2
	deltaAngle = (deltaDistanceRight - deltaDistanceLeft) / wheelBase

	dX = deltaDistance * math.cos(positionPrev[2] + deltaAngle/2)
	dY = deltaDistance * math.sin(positionPrev[2] + deltaAngle/2)

	position[0] = positionPrev[0] + dX
	position[1] = positionPrev[1] + dY
	position[2] = (positionPrev[2] + deltaAngle) % (2*math.pi)

	# Fixing encoder angle to global system.
	if position[2] < 0:
		position[2] = 2*math.pi + position[2]

def callback(data):
	# rospy.loginfo(rospy.get_caller_id() + "Recieved: ", str(data.data))
	# print('IR Left: ' + str(data.IR_left) + ' IR Center: ' + str(data.IR_center) + ' IR Right: ' + str(data.IR_right) + ' ENC Left: ' + str(data.enc_left) + ' ENC Right: ' + str(data.enc_right))
	global position
	global positionPrev
	global distanceTraveledPrev
	global distanceTraveled

	deadReckoning(-(data.enc_right), data.enc_left)

	odometryMsg.pos_x = position[0]
	odometryMsg.pos_y = position[1]
	odometryMsg.pos_a = position[2]
	odometryMsg.ros_time = rospy.get_rostime()

	# rospy.loginfo('Enc L, Enc R = %s, %s', lTicks, rTicks)
	rospy.loginfo('X, Y, A = %s, %s, %s', str(position[0]), str(position[1]), str(position[2]))
	# ("X = " + str(position[0]) + ", Y = " + str(position[1]) + " A = " + str(position[2]))
	
	# Set prev
	positionPrev = position
	distanceTraveledPrev = distanceTraveled

	pub.publish(odometryMsg)

	# rate = rospy.Rate(2) # 1hz
	# rate.sleep()

def serial():
	rospy.init_node('serial', anonymous=True)
	rospy.Subscriber('g5_sensor_channel', sensorData, callback)

	# spin() simply keeps python from exiting until this node is stopped
	rospy.spin()


if __name__ == '__main__':
	try:
		toggle = 0
		serial()
	except rospy.ROSInterruptException:
		pass
