#! /usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from math import exp, sqrt, pi, isnan, isinf
import time
import random


speed = 0.4

pastTime = 0

def mean(values):
	total = 0
	i = 0
	for v in values:
	        if data != 0:
	            total += v
	            i += 1;
	result = total / i
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
	
		side_left_sect = mean(data_sns[0:50])
		first_left_sector = mean(data_sns[50:150])
		second_left_sector = mean(data_sns[150:225])
		front_sector = mean(data_sns[225:275])
		second_right_sector = mean(data_sns[275:350])
		first_right_sector = mean(data_sns[350:450])
		side_right_sect = mean(data_sns[450:500])
		
		if side_left_sect < 0.3:
			print "SIDE L"
			base_data.linear.x = 0.1
			base_data.angular.z = 1
		elif first_left_sector < 0.5:
			print "LEFTLEFT"
			base_data.linear.x = 0.1
			base_data.angular.z = 3
		elif second_left_sector < 0.75:
			print "LEFT"
			base_data.linear.x = 0.1
			base_data.angular.z = 10
		elif front_sector < 1:
			diff = mean(data_sns[0:200]) - mean(data_sns[300:500])
			base_data.linear.x = 0.1
			print "FRONT"
			if diff < 0:	
				base_data.angular.z = 15
			else:
				base_data.angular.z = -15
		elif second_right_sector < 0.75:
			print "RIGHT"
			base_data.linear.x = 0.1
			base_data.angular.z = -10
		elif first_right_sector < 0.5:
			print "RIGHTRIGHT"
			base_data.linear.x = 0.1
			base_data.angular.z = -3
		elif side_right_sect < 0.3:
			print "SIDE R"
			base_data.linear.x = 0.1
			base_data.angular.z = -1
	
		pub.publish(base_data)
		
		pastTime = currentTime


if __name__ == '__main__':
	rospy.init_node('reactive_mover_node')
	rospy.Subscriber('base_scan', LaserScan, callback)
	pub = rospy.Publisher('cmd_vel', Twist, queue_size=100)
	rospy.spin()

	
