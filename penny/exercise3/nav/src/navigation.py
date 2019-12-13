#!/usr/bin/env python

import rospy
import numpy
import actionlib
from math import radians, degrees
from actionlib_msgs.msg import *
from geometry_msgs.msg import Point
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal


class map_navigation():
    
    def definition(self):
        global checkpoints	
        global current_point
        checkpoints = [[1.2341,4.0208,-0.8185,0.5743], [-3.0885,3.1950,0.1631,0.9865], [8.0631,0.9918,0.9772,0.2121], [-5.1741,6.3072,-0.2029,0.9791]]
        current_point = 0

    def update_checkpoint(self, goalReached):
        global current_point
        global checkpoints
        if goalReached:
            if current_point < len(checkpoints):
                current_point += 1
            else:
                current_point = 0
            return True
        else:
            return False

    def moveToGoal(self, xGoal, yGoal, zOrientGoal, wOrientGoal):
	#define a client for to send goal requests to the move_base server through a SimpleActionClient
        #print("Client before")
        ac = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        #print("Client after")
        #wait for the action server to come up
        while(not ac.wait_for_server(rospy.Duration.from_sec(5.0))):
            rospy.loginfo("Waiting for the move_base action server to come up")

        goal = MoveBaseGoal()

        #set up the frame parameters
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()

        # moving towards the goal*/
        goal.target_pose.pose.position =  Point(xGoal,yGoal,0)
        goal.target_pose.pose.orientation.x = 0.0
        goal.target_pose.pose.orientation.y = 0.0
        goal.target_pose.pose.orientation.z = zOrientGoal
        goal.target_pose.pose.orientation.w = wOrientGoal

        rospy.loginfo("Sending goal location ...")
        ac.send_goal(goal)

        ac.wait_for_result()

        if(ac.get_state() ==  GoalStatus.SUCCEEDED):
            rospy.loginfo("You have reached the destination")
            return True
        else:
            rospy.loginfo("The robot failed to reach the destination")
            return False



    
    def __init__(self):
        self.definition()
        global checkpoints	
        global current_point
        bl = True

	rospy.init_node('map_navigation', anonymous=False)
	
        while(bl):
            print("Going to...")
            print(checkpoints[current_point][0])
            print(checkpoints[current_point][1])
            self.goalReached = self.moveToGoal(checkpoints[current_point][0], checkpoints[current_point][1], checkpoints[current_point][2], checkpoints[current_point][3])
            bl = self.update_checkpoint(self.goalReached)
      
    def shutdown(self):
	rospy.loginfo("Quit program")
	rospy.sleep()


       
	     
                

if __name__ == '__main__':
    try:
        rospy.loginfo("You have reached the destination")
        map_navigation()
        rospy.spin()

    except rospy.ROSInterruptException:
        rospy.loginfo("map_navigation node terminated.")
               
    
    
    
