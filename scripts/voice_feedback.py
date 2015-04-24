#!/usr/bin/env python
import rospy
from sound_play.msg import SoundRequest
from sound_play.libsoundplay import SoundClient
from std_msgs.msg import String

def main():
    pub = rospy.Publisher('robotsound', SoundRequest, queue_size=10)
    rospy.sleep(0.1)
    msg = SoundRequest()
    msg.sound = -3
    msg.command = 1
    msg.arg2 = 'voice_kal_diphone'
    while not rospy.is_shutdown():
        msg.arg = raw_input("Enter Command: ")
        if msg.arg=="exit":
            return
        pub.publish(msg)
if __name__ == '__main__':

    #voice = 'voice_kal_diphone'
    #s = "No Drone"
    #soundhandle.say(s,voice)

    rospy.init_node('pub_sound_node', anonymous = True)
    main()
    #soundhandle = SoundClient()
    
    

    
    

