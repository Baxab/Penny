#! /usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from math import exp, sqrt, pi, isnan, isinf
import time


speed = 0.4
max_range = 5.6
angle = 0.8
pastTime = 0

detection_arc_start = 125 
detection_arc_end = 375
detection_distance = 0.5
object_size = 5


def mean(values):               
	total = 0
	i = 0
	for v in values:
		if v != 0:
			total += v
        		i += 1;
	result = total / i
	return result
    
def turn(sensor_data):
    right_average = mean(sensor_data[:200])
    left_average = mean(sensor_data[300:]) 
    result = left_average - right_average
    if result < 0:
        result = -(angle)
    else:
        result = angle
    return result

def callback(sensor_data):

	global pastTime
	currentTime = time.time()
	if (currentTime - pastTime) > 0.1:
		#print "calledback"
		data_sns = []
	
		for num, data in enumerate(sensor_data.ranges):
			if isnan(data):
				data_sns.append(0)
			elif isinf(data):
				data_sns.append(max_range)
			else:
				data_sns.append(data)
	
		base_data = Twist()
	
		base_data.linear.x = speed
		found = False
	
		for i in range(len(data_sns[detection_arc_start:detection_arc_end]) - object_size):
			object_count = 0
			for j in range(object_size):
				if(data_sns[detection_arc_start+i+j]) < detection_distance:
					object_count += 1
			if object_count == object_size:
				base_data.linear.x = 0.1
				turn_value = turn(data_sns)
				base_data.angular.z = turn_value
				found = True			
				break
			if found:
				break		
	
        	pub.publish(base_data)

        	pastTime = currentTime


if __name__ == '__main__':
    rospy.init_node('reactive_mover_node')
    rospy.Subscriber('base_scan', LaserScan, callback)
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=100)
    rospy.spin()

    
