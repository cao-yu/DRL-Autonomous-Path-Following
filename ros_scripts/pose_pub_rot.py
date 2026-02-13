#!/usr/bin/env python

import rospy
import tf
from geometry_msgs.msg import Twist


yaw_ref = 0.7854 # rad
omega_max = 0.3 # rad/s
Kp = 0.5


if __name__ == '__main__':
	rospy.init_node('tf_listener_node', anonymous=True)
	cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
	tf_listener = tf.TransformListener()
	rate = rospy.Rate(20)
	rospy.sleep(1)
	while not rospy.is_shutdown():
		try:
			tf_listener.waitForTransform('map', 'base_footprint', rospy.Time(), rospy.Duration(1.0))
			(trans, rot) = tf_listener.lookupTransform('map', 'base_footprint', rospy.Time())
			euler = tf.transformations.euler_from_quaternion(rot)
			# x, y, yaw
			print(trans[0], trans[1], euler[2])
		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
			rospy.logwarn("Failed to get tf transformation")
		
		error = yaw_ref - euler[2]
		omega = Kp * error
	
		if omega > omega_max:
		    omega = omega_max
		elif omega < -omega_max:
		    omega = -omega_max
		    
		cmd_vel = Twist()
		if abs(error) < 0.002:
		    cmd_vel.linear.x = 0.0
		    cmd_vel.angular.z = 0.0
		else:
		    cmd_vel.linear.x = 0.0
		    cmd_vel.angular.z = omega
		cmd_pub.publish(cmd_vel)

		rate.sleep()
