#! /usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from math import exp, sqrt, pi, isnan, isinf
import time
import random

speed = 0.4#0.2
max_range = 3
max_turn = 45
angle = 1
detection_arc_start = 125
detection_arc_end = 375
detection_distance = 0.75#0.5
object_size = 5

count = 0
turn_left = [0,0,0,0,0,0,0,0,0,0]
turn_right = [0,0,0,0,0,0,0,0,0,0]
already_turned = True 
 
def mean(sensor_values):
    total = 0
    for data in sensor_values:
        total += data
    result = total / len(sensor_values)
    return result

def turn(sensor_data):
    right_average = mean(sensor_data[:200])
    left_average = mean(sensor_data[300:])
    result = left_average - right_average
    print "Right: " + str(right_average)
    print "Left: " + str(left_average)
    print "Result" + str(result)
    
    if result < 0:
        result = -(angle)
    
    else:
        result = angle
    return result

    
pastTime = 0
def callback(sensor_data):
    currentTime = time.time()
    if (currentTime - pastTime) > 0.1:
        print "calledback"

        data_sns = []

        for num, data in enumerate(sensor_data.ranges):
            if isnan(data):
                data_sns.append(0)
            elif isinf(data):
                data_sns.append(max_range)
            else:
                data_sns.append(data)

        #print(data_sns[200:300])

        base_data = Twist()
	
	#if mean(data_sns[225:275]) > 1:
	base_data.linear.x = speed+0.4
	#else:
	#	base_data.linear.x = speed  
    
        for i in range(len(data_sns[detection_arc_start:detection_arc_end]) - object_size):
            object_count = 0
            for j in range(object_size):
                if(data_sns[detection_arc_start+i+j]) < detection_distance:
                    object_count += 1
            if object_count == object_size:
                base_data.linear.x = 0
                turn_value = turn(data_sns)
                base_data.angular.z = turn_value
                break

        global count

      #  if mean(data_sns[50:200]) > 3.5:
	#	print "RRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRR"
       	 #   	base_data.angular.z = -1
	#elif mean(data_sns[300:450]) > 3.5:
	 #   	print "LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL"
	#	base_data.angular.z = 3
	

        pub.publish(base_data)
        global pastTime
        pastTime = currentTime
        print "end"

if __name__ == '__main__':
    rospy.init_node('reactive_mover_node')
    rospy.Subscriber('base_scan', LaserScan, callback)
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=100)
    rospy.spin()

    
