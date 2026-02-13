#!/usr/bin/env python

import rospy
import tf


if __name__ == '__main__':
	rospy.init_node('tf_listener_node', anonymous=True)
	tf_listener = tf.TransformListener()
	rospy.sleep(1)

	rate = rospy.Rate(20)
	while not rospy.is_shutdown():
		try:
    			tf_listener.waitForTransform('map', 'base_footprint', rospy.Time(), rospy.Duration(1.0))
    			(trans, rot) = tf_listener.lookupTransform('map', 'base_footprint', rospy.Time())
    
    			euler = tf.transformations.euler_from_quaternion(rot)
    			# x, y, yaw
    			print(trans[0], trans[1], euler[2])
		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
    			rospy.logwarn("Failed to get tf transformation")

	rate.sleep()
