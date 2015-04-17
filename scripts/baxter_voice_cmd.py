from __future__ import print_function

# Import the necessary Python modules
import sys
import math
import numpy as np
import time

# rospy - ROS Python API
import rospy
from  std_msgs.msg import String


def voice_cb(data):
    if data.string == "POINT TO THE QUAD":
        return
    elif data.string == "FIND THE QUAD":
        return
    elif data.string == "THANK YOU":
        return
    elif data.string == "GOODBYE BAXTER":
        return
    elif data.string == "HELLO BAXTER":  
        return

'''
TURN ON THE QUAD
RAISE THE QUAD
LOWER THE QUAD
LAND THE QUAD
TURN OFF THE QUAD
FIND THE QUAD
FIND BUZZ
POINT TO THE QUAD
'''



def main():
    rospy.init_node('baxter_voice_cmds')
    rospy.Subscriber("recognizer_1/output", String, voice_cb)

if __name__ == '__main__':
    main()
