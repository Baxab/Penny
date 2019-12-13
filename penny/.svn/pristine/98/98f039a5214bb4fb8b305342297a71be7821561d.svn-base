#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Revision $Id: listener.py 5263 2009-07-17 23:30:38Z sfkwc $

## Simple talker demo that listens to std_msgs/Strings published 
## to the 'chatter' topic

import rospy
from std_msgs.msg import String

def param_talker():
    rospy.init_node('param_talker')

   
    # set some parameters
    rospy.loginfo('setting parameters...')
    rospy.set_param('list_of_floats', [1., 2., 3., 4.])
    rospy.set_param('bool_True', True)
    rospy.set_param('~private_bar', 1+2)
    rospy.set_param('to_delete', 'baz')

    rospy.set_param('/move_base_node/global_costmap/height',4000)
    rospy.set_param('/move_base_node/global_costmap/width',4000)
    rospy.set_param('/move_base_node/local_costmap/inflation_layer/inflation_radius', 0.2)
    rospy.set_param('/move_base_node/global_costmap/inflation_layer/inflation_radius', 0.2)
    rospy.set_param('/move_base_node/global_costmap/footprint', [[0.4,0.4],[0.4,-0.4],[-0.4,0.4],[-0.4,-0.4]])
    rospy.set_param('/move_base_node/local_costmap/footprint', [[0.4,0.4],[0.4,-0.4],[-0.4,0.4],[-0.4,-0.4]])
    
    



    rospy.loginfo('...parameters have been set')

    
    # publish the value of utterance repeatedly
   # pub = rospy.Publisher(topic_name, String, queue_size=10)
   # while not rospy.is_shutdown():
    #    pub.publish(utterance)
     #   rospy.loginfo(utterance)
      #  rospy.sleep(1)
        
if __name__ == '__main__':
    try:
        param_talker()
    except rospy.ROSInterruptException: pass
    
