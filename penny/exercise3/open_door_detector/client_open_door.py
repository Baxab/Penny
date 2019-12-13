#!/usr/bin/env python

import sys
import rospy
import math
from open_door_detector.srv import *
from std_msgs.msg import String, Bool, Float32
from controller.msg import ControllerTransition

def definition():
    global STATE_NAME
    global aperture_angle
    global wall_distance
    global min_door_width
    STATE_NAME = "state_open_door_detection"
    aperture_angle = (2*math.pi)/3
    wall_distance = 1.5
    min_door_width = 0.2#0.2 #0.40

def callback(change_state_data):
    global STATE_NAME
    global aperture_angle
    global wall_distance
    global min_door_width

    if (STATE_NAME in change_state_data.state):
        rospy.loginfo("Requesting %s %s %s"%(aperture_angle, wall_distance, min_door_width))
        res = detect_door(aperture_angle, wall_distance, min_door_width)
        rospy.loginfo(res)
        if(res.data):
            rospy.loginfo("OPEN DOOR")
        else:
            rospy.loginfo("CLOSED DOOR")
        pub.publish(res.data)
    
def detect_door(aperture_angle, wall_distance, min_door_width):
    rospy.loginfo("Waiting for service...")
    rospy.wait_for_service('detect_open_door')
    #try:
    detect_open_door_srv = rospy.ServiceProxy('detect_open_door', detect_open_door)
    resp1 = detect_open_door_srv(aperture_angle, wall_distance, min_door_width)
    return resp1.isOpen
    #except rospy.ServiceException, e:
    #    print "Service call failed: %s"%e


if __name__ == "__main__":
    rospy.init_node('open_door_client')

    definition()

    rospy.Subscriber('state_machine/control', ControllerTransition, callback)

    pub = rospy.Publisher('state_open_door_detection', Bool, queue_size=10)
    
    rospy.spin()

