#! /usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from math import exp, sqrt, pi, isnan, isinf
import time
import random

speed = 0.5
max_range = 5.6
max_turn = 90

detection_arc_start = 100
detection_arc_end = 400
detection_distance = 0.5
object_size = 5

count = 0
i = 0

right_avg_arr = [0,0,0,0,0,0,0,0,0,0]
left_avg_arr = [0,0,0,0,0,0,0,0,0,0]

def mean(sensor_values):
    total = 0
    for data in sensor_values:
        total += data
    result = total / len(sensor_values)
    return result

def turn(sensor_data):
    right_average = mean(sensor_data[:250])
    left_average = mean(sensor_data[250:])
    result = left_average - right_average
    #print "Right: " + str(right_average)
    #print "Left: " + str(left_average)
    #print "Result" + str(result)
    result = result / max_range
    result = result * max_turn
    #print "Turn Result" + str(result)
    return result

def callback(sensor_data):
    #print "calledback"

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

    base_data.linear.x = speed

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
    global i
    right_avg = mean(data_sns[:200])
    left_avg = mean(data_sns[300:])
    right_avg_arr[count] = right_avg
    left_avg_arr[count] =  left_avg
    #print left_avg
    #print right_avg
    if i >= 9:
        tot_right = 0
        tot_left = 0
    
        for x in right_avg_arr:
            tot_right = tot_right + x
        res_right = tot_right/len(right_avg_arr)

        for y in left_avg_arr:
            tot_left = tot_left + y
        res_left = tot_left/len(left_avg_arr)

        if res_right > 3.9 or res_left > 3.9:
            diff_means = res_right - res_left
            print diff_means    
            if (diff_means >= 0):
                base_data.angular.z = -45
            else:
                base_data.angular.z = 45    

    if count == 9:
	count = 0
    else:
    	count = count+1

    i = i+1
    

    pub.publish(base_data)

if __name__ == '__main__':
    rospy.init_node('reactive_mover_node')
    rospy.Subscriber('base_scan', LaserScan, callback)
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=100)
    rospy.spin()

    
