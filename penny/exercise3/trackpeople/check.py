#!/usr/bin/env python

import roslib
import rospy
import tf
import time
import math
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from controller.msg import ControllerTransition

def turn(y_pos, lr_lim):                                #The function returns a fixed turn value ('angle') for Penny  
    if y_pos < -lr_lim or y_pos > lr_lim:               #to turn if the detected person is not in the centre of the kinect's
       result = math.copysign(angle,y_pos)              #view, which is bounded by 'lr_lim' to either sides from the centre.
    else:
        result = 0
#    if lr < 0:
#        result = -turn
#    else:
#        result = turn
    return result


def follow_person(userid):
    global check_fb
    global check_lr
    global i
    global user
    try:
        print listener.frameExists('/base_link')
        frame_data = []
        listener.waitForTransform(BASE_FRAME, "/%s_%d" % (FRAME, userid), rospy.Time(), rospy.Duration(find)) #Collects the 'translation' and 
        trans, rot = listener.lookupTransform(BASE_FRAME, "/%s_%d" % (FRAME, userid), LAST)                   #'rotation' data from tracker node.
        frame_data.append(trans)        #Collects the translation distances from the tracker node in frame_data (Rotation is not required).
        fb, lr, _ = frame_data[0]       #'fb'is distance of tracked person from kinect. 'lr' is lateral distance from the centre of its view.
        turn_angle = turn(lr,lr_follow_lim)     #Turn value for turning (if required) while following a person.
        if fb == check_fb and lr == check_lr:   #Checks if person is lost (two consecutive distance values from the tracker node will be equal)
            base_data.linear.x = 0                          #Penny stops linear motion if tracked person is lost to a side
            turn_angle = 0.7*turn(lr,lr_facerec_lim)        #and spins in the suitable direction to find lost person.
            i += 1
            if i < timeout:                                 #Timeout for finding lost person (and for spinning).
                for new_user in range(0,6):                                         #Compensates for the tracker node assigning the tracked
                    if listener.frameExists('torso_'+str(new_user)):                #person as a new person.
                        user = new_user                                             #Assigns the newly tracked person as the person to be followed.
                if lr > 0:
                    print 'Person Lost to the Left!'
                else:
                    print 'Person Lost to the Right!'
                print 'Sounding alarm in ' + str((timeout - i)*loop_sleep) + 's'    #Calls the 'alarm' node if lost person is not found in time.
            else:
                print 'Person Lost... Sounding Alarm...'                            #Calls 'alarm' node.
                state_pub.publish(False)
                return False
            #return False
        else:  
            i = 0                    
            if fb > fb_lim:                         #Follows the detected person till he/she is close enough for face recognition.
                base_data.linear.x = speed*fb       #Speed increases with distance from person.
                print "Following user " + str(userid) + "..."
            else:
                base_data.linear.x = 0                      
                if fb < too_close:                  #Checks if the person is too close for proper face recognition.
                    print "Please take a step back."
                else:
                    print 'Stay Still for Verification'
                    if check_fb < fb_lim:   #Checks if two consecutive values (distance from kinect) are within range for face recognition.
                        if lr < lr_facerec_lim and lr > -lr_facerec_lim:    #Checks if the person is in the centre of the frame.
                            print "Beginning Face Recognition for person " + str(userid) + "..."    
                            state_pub.publish(True)                         #Calls face recognition node.
                            return False
                        else:
                            turn_angle = 0.2*turn(lr,lr_facerec_lim)    #Penny rotates to bring person to the centre of the frame.
                            print 'Turning towards person ' + str(userid)
                    else:    
                        time.sleep(2)       #Waits 2 seconds to see if the person stays within range for face recognition.
        base_data.angular.z = turn_angle/fb #Penny turns in the required direction at a value depending on the distance from person.
        print str(check_fb) + '<------------fb------------->' + str(fb)
        print str(check_lr) + '<------------lr------------->' + str(lr)
        check_fb = fb                       #For comparing two consecutive values for 'fb' and 'lr' from the tracker node, 
        check_lr = lr                       #so as to determine whether person is lost.
        print 'fb = ' + str(fb)
        print 'lr = ' + str(lr)
        print 'speed = ' + str(base_data.linear.x)
        print 'turn = ' + str(base_data.angular.z)
        pub.publish(base_data)
        return frame_data
    except (tf.LookupException,             #Raises error message for 'Exception Errors' from tracker node.
            tf.ConnectivityException,
            tf.ExtrapolationException):
        raise IndexError

def callback(data):
    if data.state == "state_follower":
        global i
        global user
        user = int(data.data)
        print "Following"
        i = 0
        keep_going = 1
        while keep_going != False:              #The 'follow_person' function is called to follow 'user' unless the function returns 'False'. 
            keep_going = follow_person(user)    #The function returns 'False' only when the tracked person is lost or close enough for 
            time.sleep(loop_sleep)              #face recognition.
        print "Stopped following"

        

if __name__ == "__main__":
    rospy.init_node('follower_node')
    rospy.Subscriber('state_machine/control', ControllerTransition, callback)
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=100)
    state_pub = rospy.Publisher('state_follower', Bool, queue_size=100)
    listener = tf.TransformListener()

    #Assigning parameters and variables
    BASE_FRAME = '/openni_depth_frame'
    FRAME = 'torso'
    LAST = rospy.Duration()
    name='kinect_listener'     
    loop_sleep = 0.5
    find = 10
    i = 0
    fb_lim = 1.5
    lr_follow_lim = 0.3
    lr_facerec_lim = 0.1    
    angle = 0.6
    speed = 0.2
    too_close = 0.5
    base_data = Twist()
    fb = 0
    lr = 0
    check_fb = 100
    check_lr = 100
    timeout = 30
    
    rospy.spin()
