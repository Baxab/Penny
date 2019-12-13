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
	
		arc_start = 175
		arc_end = 325
		detection_distance = 0.75
		for i in range(len(data_sns[arc_start:arc_end]) - 5):
		        if data_sns[arc_start+i] < detection_distance:
		                if data_sns[arc_start+i+1] < detection_distance:
		                        if data_sns[arc_start+i+2] < detection_distance:
		                                if data_sns[arc_start+i+3] < detection_distance:
						        if data_sns[arc_start+i+4] < detection_distance:
						                if data_sns[arc_start+i+5] < detection_distance:
						                        base_data.linear.x = 0
									base_data.angular.z = turn(data_sns)
									break
									#print "Turning"

		pub.publish(base_data)
		global pastTime
		pastTime = currentTime

if __name__ == '__main__':
	rospy.init_node('reactive_mover_node')
	rospy.Subscriber('base_scan', LaserScan, callback)
	pub = rospy.Publisher('cmd_vel', Twist, queue_size=100)
	rospy.spin()

	
