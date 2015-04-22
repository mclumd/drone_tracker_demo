#!/usr/bin/env python
import sys
import rospy
from sound_play.msg import SoundRequest
from sound_play.libsoundplay import SoundClient

if __name__ == '__main__':
    rospy.init_node('pub_sound', anonymous = True)
    soundhandle = SoundClient()
    rospy.sleep(1)
    voice = 'voice_kal_diphone'
    s = "No Drone"
    soundhandle.say(s,voice)
    #pub = rospy.Publisher('robotsound', SoundRequest, queue_size=1)
    #msg = SoundRequest()
    #msg.sound = -3
    #msg.command = 1
    #msg.arg = "Could not find the drone"
    #msg.arg2 = "Could not find the drone"
    #pub.publish(msg)
    

