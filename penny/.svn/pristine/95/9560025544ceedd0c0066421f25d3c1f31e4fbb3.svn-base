#! /usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from math import exp, sqrt, pi, isnan, isinf
import time


speed = 0.4     # Setting speed of the Robot
max_range = 4.5#5.6   # Maximum Range that the laser can detect
angle = 0.7       # Angle of rotation
angle_too_close = 0.85
pastTime = 0    # Past time for calculating time intervals

detection_arc_start = 125               # The values for the start and end of the arc for the vision in the front 
detection_arc_end = 375                 # and minimum detection distance have been set after experimentation.
detection_distance = 0.85                # Distance at which an object will be detected
too_close_distance = 0.6
too2_close_distance = 0.4
object_size = 5                         # The minimum size of an object to be seen by the detector


# Function to calculate the mean obtained from the array of LaserScan values 
def mean(sensor_values):               
    total = 0
    i = 0
    for data in sensor_values:
        if data != 0:
            total += data
            i += 1;
    result = total / i
    return result
    
# Function to decide the angle of rotation
def turn(sensor_data):
    right_average = mean(sensor_data[30:212]) # The array of the LaserScan values has been split into three sectors. The first 200 values
    left_average = mean(sensor_data[300:482])  # represent the right sector. The last 200 values represent the left sector. The direction 
    result = left_average - right_average
    if result < 0:
        result = -(angle)
    else:
        result = angle
    return result


def turn_too_close(sensor_data):
    right_avg = mean(sensor_data[10:30])
    left_avg = mean(sensor_data[482:502])
    result = left_avg - right_avg
    if result < 0:
        result = -(angle_too_close)
    else:
        result = angle_too_close
    return result    
    
def turn_too_close_front(sensor_data):
    right_avg = mean(sensor_data[:212])
    left_avg = mean(sensor_data[300:])
    result = left_avg - right_avg
    if result < 0:
        result = -(angle_too_close)
    else:
        result = angle_too_close
    return result


def callback(sensor_data):
    currentTime = time.time()
    if (currentTime - pastTime) > 0.1:  

        print "###############################"

        data_sns = []
        for num, data in enumerate(sensor_data.ranges):  # Setting zero for values that could not be a number,setting maximum range 
            if isnan(data):                          # for values that could be greater than maximum range and putting it into
                data_sns.append(0)               # a new array.
            elif isinf(data):
                data_sns.append(max_range)
            else:
                data_sns.append(data)
    
        base_data = Twist()                              # Setting the linear motion to the required speed.
        base_data.linear.x = speed  
        
        too_close = False
        too2_close = False
        found = False





    
        for i in range(len(data_sns[detection_arc_start:detection_arc_end]) - object_size):
            object_count = 0
            for j in range(object_size):
                if(data_sns[detection_arc_start+i+j]) < detection_distance:
                    object_count += 1
                    if (data_sns[detection_arc_start+i+j]) < too_close_distance:
                        too_close = True
                    if (data_sns[detection_arc_start+i+j]) < too2_close_distance:
                        too_close = False
                        too2_close = True
                if object_count == object_size:
                    if too_close:
                        if i > 226 and i < 286:
                            base_data.linear.x = 0
                            turn_value = turn_too_close_front(data_sns)
                            found = True    
                            print "FRONTFRONTFRONTFRONTFRONT"
                        else:
                            base_data.linear.x = 0.1
                            turn_value = turn_too_close(data_sns)
                            found = True
                            print "SIDESIDESIDESIDESIDESIDE"    
                    elif too2_close:
                        base_data.linear.x = 0
                        turn_value = turn_too_close_front(data_sns)
                        found = True
                        print "TOO_CLOSE_TOO_CLOSE_TOO_CLOSE"                
                    else:
                        base_data.linear.x = 0.3
                        turn_value = turn(data_sns)
                        found = True
                    base_data.angular.z = turn_value
                    break
            if found:
                break

	if too_close or too2_close:
	    too_close = True
	    too2_close = True

        if not (too_close) and not (too2_close):
    
            right_side_avg = mean(data_sns[10:60])
            left_side_avg = mean(data_sns[462:502])
        
            #print right_side_avg
            #print left_side_avg    
        
            if (right_side_avg > 1 and left_side_avg > 1):
                
                print "ENTRATOENTRATOENTRATOENTRATO"
                
                right_avg = mean(data_sns[127:212])
                left_avg = mean(data_sns[297:382])
    
                print right_avg
                print left_avg
            
                if right_avg > 3.3 and left_avg > 3.3: #3.9 for sim
                    base_data.angular.z = -0.6
                elif right_avg > 3.3 or left_avg > 3.3: #3.9 for sim
                    diff_means = right_avg - left_avg
                    if (diff_means >= 0):
                    	base_data.angular.z = -0.6
                    else:
                    	base_data.angular.z = 0.6  

    




	pub.publish(base_data)              #Publish
	global pastTime
        pastTime = currentTime



if __name__ == '__main__':
    rospy.init_node('reactive_mover_node')
    rospy.Subscriber('base_scan', LaserScan, callback)
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=100)
    rospy.spin()

    
