#! /usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from math import exp, sqrt, pi, isnan, isinf
import time
import random

speed = 0.5
max_range = 5.6
max_turn = 45

i = 0
count = 0
right_avg_arr = [0,0,0,0,0,0,0,0,0,0]
left_avg_arr = [0,0,0,0,0,0,0,0,0,0]

def mean(sensor_values):
	total = 0
	ii = 0
	for data in sensor_values:
	        if data != 0:
	            total += data
	            ii += 1;
	result = total / ii
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

#pastTime = 0
pastTime2 = 0
def callback(sensor_data):
	#currentTime = time.time()
	#if (currentTime - pastTime) > 0.1:
	#print "calledback"
	global pastTime2
    	currentTime = time.time()
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
#		print diff
		#if (diff>-0.3 and diff<0):
		#	base_data.angular.z = 15
		#elif (diff>=0 and diff<0.3):
		#	base_data.angular.z = -15
		#else: 
			
	#	if (diff>-0.3 and diff<0.3):
	#		base_data.angular.z = 180
	#		#time.sleep(0.5)
	#	else:
		if diff < 0:	
			base_data.angular.z = 15
		else:
			base_data.angular.z = -15
		#m1 = mean(data_sns[100:200])		
		#m2 = mean(data_sns[300:400])
		#print m1
		#print m2
		#if  (m1 < 0.75 and m2 < 0.75): 
		#	print "######################################"
		#	global speed
		#	speed = -0.2
		#	while m1 < 1 and m2 < 1:
		#		base_data.linear.x = -0.2
		#		base_data.angular.z = 90
				

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

	
	#if (currentTime - pastTime2) > 5:
	#	sign = 0
        #	for r in range(0,10):
           # if count<9
           #     print "COUNT LESS THAN 9"
           #     turn_left[count] = mean(data_sns[50:150])
           #     turn_right[count] = mean(data_sns[350:450])
           #     count = count+1
           # elif count == 9:
           #     print "COUNT EQUALS 9"
           #     turn_left[count] = mean(data_sns[50:150])
           #     turn_right[count] = mean(data_sns[350:450])
           #     count = 0

            
		        #print "HEEELLOOOOOOW"
		        #global count
		        
	#		if r == 0:
	#	        
	#	        	if mean(data_sns[50:150]) > 3:
	#	        	    	print "RRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRR"
	#	        	    	base_data.angular.z = -10
	#	           		sign = -1
	#	               
	#			elif mean(data_sns[350:450]) > 3:
	#			    	print "LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL"
	#			    	base_data.angular.z = 10
	#				sign = 1				
#
#			else:
#				
#				print sign
#				base_data.angular.z = 10*sign


#		pastTime2 = currentTime

		
	#time.sleep(0.1)



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
	#global pastTime
#	#pastTime = currentTime

if __name__ == '__main__':
	rospy.init_node('reactive_mover_node')
	rospy.Subscriber('base_scan', LaserScan, callback)
	pub = rospy.Publisher('cmd_vel', Twist, queue_size=100)
	rospy.spin()

	
