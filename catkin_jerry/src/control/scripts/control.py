#!/usr/bin/env python

# Receives messages from matlab and publishes them to a topic over rosserial.

import rospy
import numpy as np
from robot_msgs.msg import ctrlData
from std_msgs.msg import String

ctrlMsg = ctrlData()
pub = rospy.Publisher('g5_arduino_ctrl_channel_jerry', ctrlData, queue_size=1)
# pub = rospy.Publisher('g5_arduino_ctrl_channel_tom', ctrlData, queue_size=1)

def callback(data):

    splitData = [x.strip() for x in data.data.split(', ')]
    ctrlMsg.SpeedLeft =  int(splitData[0])
    ctrlMsg.SpeedRight = int(splitData[1])
    ctrlMsg.State = int(splitData[2])

    # rospy.loginfo("Speed Left: %d, Speed Right: %d, State: %d", int(splitData[0]), int(splitData[1]), int(splitData[2]))

    pub.publish(ctrlMsg)

def control():
	rospy.init_node('g5_control_node', anonymous=True)
	rospy.Subscriber('g5_ctrl_channel_jerry', String, callback)
	#rospy.Subscriber('g5_ctrl_channel_tom', String, callback)

	# rate = rospy.Rate(100) # 1hz
	# rate.sleep()
	# spin() simply keeps python from exiting until this node is stopped
	rospy.spin()


if __name__ == '__main__':
    try:
        control()
    except rospy.ROSInterruptException:
	rospy.loginfo("ROSInterruptException")
        ctrlMsg.SpeedLeft = 0
        ctrlMsg.SpeedRight = 0
        ctrlMsg.State = 0
        pub.publish(ctrlMsg)
        pass
