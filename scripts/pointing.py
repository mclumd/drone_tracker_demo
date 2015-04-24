#!/usr/bin/env python
from __future__ import print_function

# Import the necessary Python modules
import sys
import math
import numpy as np
import time

# rospy - ROS Python API
import rospy

# baxter_interface - Baxter Python API
import baxter_interface

#Message carrying location of target
from geometry_msgs.msg import Point
from std_msgs.msg import String
from sound_play.msg import SoundRequest
from sound_play.libsoundplay import SoundClient

pub_audio = None
sub_cmd = None
sub_arm = None


def point_joint_angles(target):
	'''
	target is a numpy array or python list containing the 3d location of the thing to be pointed at, using robot-centered coordinates.
	'''
	
	#Center on location of the first point of rotation of the right arm
	base_joint_pos = np.array([0.07316, -0.26733, 0.31])
	target = target - base_joint_pos
	
	#set fixed angles to 0
	angles = {'right_e0': 0.0, 'right_e1': 0.0, 'right_w0': 0.0, 
	'right_w1': 0.0, 'right_w2': 0.0}
	
	#calculate variable angles
	angles['right_s0']=math.atan2(target[0], -target[1]) - np.pi/4
	angles['right_s1']=-math.atan2(target[2], np.linalg.norm(target[:2]))
	
	print("pointing: angles for joint right s0:", angles['right_s0'])
	print("pointing: angles for joint right s1:", angles['right_s1'])
	
	return angles

def check_range(target):
	'''
	target is a numpy array or python list containing the 3d location of the thing to be pointed at, using robot-centered coordinates.
	'''
	
	#Center on location of the first point of rotation of the right arm
	base_joint_pos = np.array([0.07316, -0.26733, 0.31])
	targetnew = target - base_joint_pos
	
	#calculate variable angles
	right_s0=math.atan2(targetnew[0], -targetnew[1]) - np.pi/4
	right_s1=-math.atan2(targetnew[2], np.linalg.norm(targetnew[:2]))

	return right_s0 < 1.55 and right_s0 > -1.6 and right_s1 < 1 and right_s1 > -1.35

def point_callback(data):
	limbr = baxter_interface.Limb('right')
	limbl = baxter_interface.Limb('left')
	if data.x == 0 and data.y == 0 and data.z == 0:
		rospy.loginfo("Point: got a stop command (all zeros)")
	elif check_range([data.x, data.y, data.z]):
		rospy.loginfo("Point: setting target to" + str(data))
		limbr.move_to_joint_positions(point_joint_angles([data.x, data.y, 
		data.z]), threshold = 0.05)
		#limb.move_to_joint_positions(angles) #blocking
	else:
		msg = SoundRequest()
		msg.sound = -3
		msg.command = 1
		msg.arg = 'Quad is out of pointing range'
		msg.arg2 = 'voice_kal_diphone'
		pub_audio.publish(msg)
		rospy.loginfo("Point: target out of pointing range " + str(data))

def arm_callback(data):
	if data.data == "move your right arm to the side":
		angles = {'right_e0': 0.0, 'right_e1': 0.0, 'right_w0': 0.0, 'right_w1': 0.0, 'right_w2': 0.0}
		limbr.move_to_joint_positions(angles)       
	elif data.data == "move your left arm to the side":
		angles = {'right_e0': 0.0, 'right_e1': 0.0, 'right_w0': 0.0, 'right_w1': 0.0, 'right_w2': 0.0}
		limbr.move_to_joint_positions(angles)

def start_node():
	rospy.init_node('baxter_point')
	rospy.loginfo("Reading point commands from topic " + targetTopic)
	pub_audio = rospy.Publisher('robotsound', SoundRequest, queue_size=10)
	sub_cmd = rospy.Subscriber("point_cmd", Point, point_callback)
	sub_arm = rospy.Subscriber("arm_cmd", String, arm_callback)
	rospy.spin()

def test_angle_finder():
	rospy.init_node('baxter_point')
	points = [Point(x = 5.0, y = 0.0, z = 0.0),
			  Point(x = 8.0, y = 3.0, z = 2.0),
			  Point(x = 5.0, y = -1.0, z = -3.0)]
	n = 0
	while n < 20:
		n += 1
		time.sleep(1)
		p = points[n % 3]
		print(point_joint_angles([p.x, p.y, p.z]))
		point_callback(p)
		

if __name__ == '__main__':
    try:
        start_node()
    except rospy.ROSInterruptException:
        pass
