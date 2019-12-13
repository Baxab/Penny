#!/usr/bin/env python

import roslib
import rospy
import tf
import time
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import math


BASE_FRAME = '/openni_depth_frame'
FRAME = 'torso'
LAST = rospy.Duration()
fb_lim = 0.8
lr_lim = 0.3
angle = 0.5
speed = 0.2
too_close = 0.6
#print 'im a potato11'
base_data = Twist()


detection_arc_start = 125 
detection_arc_end = 375
detection_distance = 0.5
object_size = 5


#class Kinect:

def turn(y_pos):
    if y_pos < -lr_lim or y_pos > lr_lim:
       result = math.copysign(angle,y_pos)
       
    else:
        result = 0
#        if lr < 0:
#        result = -turn
#        else:
#        result = turn
    return result

def get_posture():
    data_sns = []
	
    for num, data in enumerate(sensor_data.ranges):
        if isnan(data):
            data_sns.append(0)
        elif isinf(data):
            data_sns.append(max_range)
        else:
            data_sns.append(data)
    found = False
    for i in range(len(data_sns[detection_arc_start:detection_arc_end]) - object_size):
	    		object_count = 0
	    		for j in range(object_size):
	    			if(data_sns[detection_arc_start+i+j]) < detection_distance:
	    				object_count += 1
	    		if object_count == object_size:
	    			base_data.linear.x = 0
	    			base_data.angular.z = 0
	    			found = True			
	    			break
	    		if found:
	    			break	
    try:
        frame_data = []
        listener.waitForTransform(BASE_FRAME, "/%s_%d" % (FRAME, user), rospy.Time(), rospy.Duration(2))
        trans, rot = listener.lookupTransform(BASE_FRAME, "/%s_%d" % (FRAME, user), LAST)
        frame_data.append(trans)
        fb, lr, _ = frame_data[0]
        #lr = frame_data[1]
        print 'fb = ' + str(fb)
        print 'lr = ' + str(lr)
        turn_angle = turn(lr)
        if fb > fb_lim:
            base_data.linear.x = speed*fb
            print "Following..."
        else:
            base_data.linear.x = 0
            if fb < too_close:
                print "Please take a step back."
            else:
                print 'Stay Still'
                time.sleep(2)
                if fb > too_close and fb < fb_lim:
                    print "Beginning Face Recognition..."
                    return False
                elif fb < too_close:
                    print "Please take a step back."
                else:
                    base_data.linear.x = speed*fb
                    print "Following again..."
                    
        base_data.angular.z = turn_angle/fb
        print 'speed = ' + str(base_data.linear.x)
        print 'turn = ' + str(base_data.angular.z)
        pub.publish(base_data)
        #print frame_data
        return frame_data

    except (tf.LookupException,
            tf.ConnectivityException,
            tf.ExtrapolationException):
        #print 'im a potato2'
        raise IndexError


name='kinect_listener'     
rospy.init_node(name, anonymous=True)
listener = tf.TransformListener()
pub = rospy.Publisher('cmd_vel', Twist, queue_size=100)
user = 1
#print 'im a potatoVFDIHGDKHIFSHGHI'

            
    
#myKinect = Kinect()
keep_going = 1
while keep_going != False:
    keep_going = get_posture()
    time.sleep(0.3)


#pub = rospy.Publisher('???', ???, queue_size=100)
