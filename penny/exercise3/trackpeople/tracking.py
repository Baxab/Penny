

#!/usr/bin/env python

"""Code developed by Stephon Mikell, Jr. It is intergrated with follow.py."""

import rospy
import tf
from std_msgs.msg import Bool

def tracker():
#	"""Checks to see if what the robot detects is a person or not, then if it is
#	it will publish True to follow.py, else it will publish False."""
    rospy.init_node('tracking')
    pub = rospy.Publisher('tracking', Bool, queue_size=10)
    while not rospy.is_shutdown():
        #t = tf.Transformer(True, rospy.Duration(10.0))
        t = tf.TransformListener()
        if t.frameExists('openni_depth_frame'):
            track = True
        else:
            track = False
        rospy.loginfo("list: %s", t.getFrameStrings())
        pub.publish(track)
        rospy.sleep(1.0)

if __name__ == '__main__':
	tracker()


