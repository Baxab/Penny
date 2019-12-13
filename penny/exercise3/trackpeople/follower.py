#!/usr/bin/env python

import roslib
import rospy
import tf
import time
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import math

#class Kinect:

def turn(y_pos, lr_lim):
    if y_pos < -lr_lim or y_pos > lr_lim:
       result = math.copysign(angle,y_pos)
       
    else:
        result = 0
#        if lr < 0:
#        result = -turn
#        else:
#        result = turn
    return result

def obstacle_turn(sensor_data):
    right_average = math.mean(sensor_data[:200])
    left_average = math.mean(sensor_data[300:]) 
    result = left_average - right_average
    if result < 0:
        result = -(angle)
    else:
        result = angle
    return result

def follow_person(userid):
    global check_fb
    global check_lr
    global count
    global user
    try:
        frame_data = []
        listener.waitForTransform(BASE_FRAME, "/%s_%d" % (FRAME, userid), rospy.Time(), rospy.Duration(10))
        trans, rot = listener.lookupTransform(BASE_FRAME, "/%s_%d" % (FRAME, user), LAST)
        frame_data.append(trans)
        fb, lr, _ = frame_data[0] 
        for num, data in enumerate(sensor_data.ranges):
            if isnan(data):
                data_sns.append(0)
            elif isinf(data):
                data_sns.append(max_range)
            else:
                data_sns.append(data)
        for i in range(len(data_sns[detection_arc_start:detection_arc_end]) - object_size):
            object_count = 0
            for j in range(object_size):
                if(data_sns[detection_arc_start+i+j]) < detection_distance:
                    object_count += 1
            if object_count == object_size:
                base_data.linear.x = 0.1
                turn_angle = obstacle_turn(data_sns)
            else:
                turn_angle = turn(lr,lr_follow_lim)
                if fb == check_fb and lr == check_lr:
                    base_data.linear.x = 0
                    turn_angle = 0.7*turn(lr,lr_facerec_lim)
                    count += 1
                    if count != timeout:
                        for new_user in range(0,6):
                            if listener.frameExists('torso_'+str(new_user)):
                                user = new_user
                        if lr > 0:
                            print 'Person Lost to the Left!'
                        else:
                            print 'Person Lost to the Right!'
                        print 'Sounding alarm in ' + str((timeout - i)*loop_sleep) + 's' 
                    else:
                        print 'Person Lost... Sounding Alarm...'
                        return False
                    #statePub.publish(False)
                    #return False
                else:  
                    count = 0                    
                    if fb > fb_lim:
                        base_data.linear.x = speed*fb
                        print "Following user " + str(userid) + "..."
                    else:
                        base_data.linear.x = 0
                        if fb < too_close:
                            print "Please take a step back."
                        else:
                            print 'Stay Still for Verification'
                            if check_fb < fb_lim:
                                if lr < lr_facerec_lim and lr > -lr_facerec_lim:
                                    print "Beginning Face Recognition for person " + str(userid) + "..."
                                    return False
                                    #statePub.publish(True)
                                else:
                                    turn_angle = 0.5*turn(lr,lr_facerec_lim)
                                    print 'Turning towards person ' + str(userid)
                            else:    
                                time.sleep(2)
        base_data.angular.z = turn_angle/fb
        print str(check_fb) + '<------------fb------------->' + str(fb)
        print str(check_lr) + '<------------lr------------->' + str(lr)
        check_fb = fb
        check_lr = lr
        print 'fb = ' + str(fb)
        print 'lr = ' + str(lr)
        print 'speed = ' + str(base_data.linear.x)
        print 'turn = ' + str(base_data.angular.z)
        pub.publish(base_data)
        return frame_data
    except (tf.LookupException,
            tf.ConnectivityException,
            tf.ExtrapolationException):
        raise IndexError
if __name__ == "__main__":
    rospy.init_node('reactive_mover_node')
    BASE_FRAME = '/openni_depth_frame'
    FRAME = 'torso'
    LAST = rospy.Duration()
    name='kinect_listener'     
    rospy.init_node(name, anonymous=True)
    listener = tf.TransformListener()
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=100)
    keep_going = 1
    loop_sleep = 0.5
    user = 1
    count = 0
    detection_arc_start = 125 
    detection_arc_end = 375
    detection_distance = 0.5
    object_size = 5
    fb_lim = 1.5
    lr_follow_lim = 0.3
    lr_facerec_lim = 0.1    
    angle = 0.7
    speed = 0.2
    too_close = 0.5
    base_data = Twist()
    fb = 0
    lr = 0
    check_fb = 100
    check_lr = 100
    timeout = 30
    finished = False
    max_range = 5.6
    while keep_going != False:
        keep_going = follow_person(user)
        time.sleep(loop_sleep)
    




	#statePub = rospy.Publisher('???', ???, queue_size=100)
