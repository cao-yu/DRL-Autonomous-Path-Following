#!/usr/bin/env python

# run "roslaunch tarkbot_nav nav.launch" before this script
# added:
# maximum velocity of each wheel is set

import rospy
import tf
import math
import numpy as np
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

from arc_reference_path import ArcReferencePath

import csv
import os
from datetime import datetime


# initial s parameters
sn = 0.0 # nearest
last_sl = 0.2

DEFAULT_L = 0.2 # [m] look-ahead distance
DEFAULT_v_ref = 0.4 # [m/s] reference velocity

v_max = 0.4 # [m/s] set maximum velocity
omega_max = 1.0 # [rad/s] set maximum velocity

linear_x = 0.0 # [m/s] measured linear velocity
angular_z = 0.0 # [rad/s] measure rotational velocity

# File to save the data
current_datetime = datetime.now()
date_time_str = current_datetime.strftime("%Y%m%d_%H%M")
data_file = f"log_pp_{date_time_str}.csv"


def pure_pursuit_control(x, y, yaw, v, target_path, last_sn, last_sl):
    sn = target_path.find_nearest_point(last_sn, x, y)
    sl = sn + DEFAULT_L

    #if sl < target_path.len:
    #    tx, ty = target_path.X(sl), target_path.Y(sl)
    #else:
    #    tx, ty = target_path.X(target_path.len), target_path.Y(target_path.len)
    #    sl = target_path.len
	
    if last_sl < sl:
        last_sl = sl
    tx, ty = target_path.X(last_sl), target_path.Y(last_sl)
    dx, dy = tx - x, ty - y	
    alpha = math.atan2(dy, dx) - yaw
    LA = math.hypot(dx, dy)
    omega = 2 * abs(v) * math.sin(alpha) / LA
    return omega, sn, last_sl


def odom_callback(msg):
    global linear_x, angular_z
    linear_x = msg.twist.twist.linear.x
    angular_z = msg.twist.twist.angular.z


def get_pose(trans, rot):
    x, y = trans[0], trans[1]
    
    quaternion = (
        rot[0],
        rot[1],
        rot[2],
        rot[3]
    )
    euler = tf.transformations.euler_from_quaternion(quaternion)
    yaw = euler[2]  
    return x, y, yaw


if __name__ == '__main__':
    
    rospy.init_node('pure_pursuit_node', anonymous=True)
    rospy.Subscriber('/tarkbot_odom_raw', Odometry, odom_callback)
    cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    
    listener = tf.TransformListener()
    rate = rospy.Rate(20.0)
    rpath = ArcReferencePath("eight")
   
    rospy.sleep(1)

    # Check if the CSV file exists, delete it if it exists, then create a new one
    if os.path.exists(data_file):
        os.remove(data_file)

    with open(data_file, mode='w') as file:
        writer = csv.writer(file)
        writer.writerow(['Time (s)', 'x (m)', 'y (m)', 'yaw (rad)', 'CTE (m)', 'Orientation Error (rad)', 'linear_x_cmd (m/s)', 'linear_x (m/s)', 'angular_z_cmd (rad/s)',  'angular_z (rad/s)', 'nearest s', 'look-ahead s'])

    base_time = None

    while not rospy.is_shutdown():
        try:
            listener.waitForTransform('map', 'base_footprint', rospy.Time(0), rospy.Duration(1.0))
            (trans, rot) = listener.lookupTransform('map', 'base_footprint', rospy.Time())
            
            # current pose and velocities
            x, y, yaw = get_pose(trans, rot)
            cv, comega = linear_x, angular_z 
            
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
            
        omega_ref, sn, last_sl = pure_pursuit_control(x, y, yaw, cv, rpath, sn, last_sl)
        cte, psi = rpath.calc_error(x, y, yaw, sn)

        # velocity constrains
        omega_ref = np.clip(omega_ref, -omega_max, omega_max)
        
        cmd_vel = Twist()
        if rpath.len - sn > 0.02:
            cmd_vel.linear.x = DEFAULT_v_ref
            cmd_vel.angular.z = omega_ref
        else:
            cmd_vel.linear.x = 0.0
            cmd_vel.angular.z = 0.0

        cmd_pub.publish(cmd_vel)

        # Save data to the file
        if base_time is None:
            base_time = rospy.Time.now().to_sec()

        relative_time = rospy.Time.now().to_sec() - base_time

        with open(data_file, mode='a') as file:
            writer = csv.writer(file)

            # Write data row if linear_x value is present
            if linear_x is not None:
                row_data = [relative_time, x, y, yaw, cte, psi, cmd_vel.linear.x, cv, cmd_vel.angular.z, comega, sn, last_sl]
                writer.writerow(row_data)

        rate.sleep()

