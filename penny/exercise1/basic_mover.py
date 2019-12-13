#! /usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from math import exp, sqrt, pi, isnan, isinf
import time
import random

speed = 0.6
max_range = 5.6 #3
max_turn = 45

detection_arc_start = 175
detection_arc_end = 325
detection_distance = 0.75#0.5
object_size = 5

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
    print "Right: " + str(right_average)
    print "Left: " + str(left_average)
    print "Result" + str(result)

    result = result / max_range
    result = result * max_turn
    if result <0:
        result = -(max_turn)-result
    elif result >0:
        result = max_turn-result
    print "Turn Result" + str(result)
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
        
        pub.publish(base_data)
        global pastTime
        pastTime = currentTime

if __name__ == '__main__':
    rospy.init_node('reactive_mover_node')
    rospy.Subscriber('base_scan', LaserScan, callback)
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=100)
    rospy.spin()

    
