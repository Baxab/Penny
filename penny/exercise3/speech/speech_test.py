#! /usr/bin/env python

import rospy
from std_msgs.msg import String
import subprocess

def speak(text):
    try:
        output = subprocess.check_output(['espeak', '-a', '100', '-v', 'f5', text], stderr=subprocess.PIPE)
    except subprocess.CalledProcessError:
        print "Speech Failed"
        return False

    return True

def callback(text):
    print "Speaking \"" + text + "\""
    speak(text)

if __name__ == '__main__':
    rospy.init_node('speech_driver_node')
    rospy.Subscriber('speak', String, callback)

    rospy.spin()
