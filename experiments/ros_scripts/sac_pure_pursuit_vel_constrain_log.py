#!/usr/bin/env python

# run "roslaunch tarkbot_nav nav.launch" before this script
# added:
# maximum velocity of each wheel is set

import rospy
import tf
import math
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

from arc_reference_path import ArcReferencePath

import csv
import os
from datetime import datetime

import onnxruntime as ort
import numpy as np



# initial s parameters
sn = 0.0 # nearest
last_sl = 0.2

DEFAULT_L = 0.2 # [m] look-ahead distance
v_max = 0.4 # [m/s] set maximum velocity
omega_max = 1.0 # [rad/s] set maximum velocity

# range for obs
min_val = np.array([[-np.inf, -np.pi, 0.0, -omega_max, -np.pi]]).astype(np.float32)
max_val = np.array([[np.inf, np.pi, v_max, omega_max, np.pi]]).astype(np.float32)

# File to save the data
current_datetime = datetime.now()
date_time_str = current_datetime.strftime("%Y%m%d_%H%M")
data_file = f"log_sac0_{date_time_str}.csv"

onnx_path = "/home/xtark/tarkbot/ros_ws/src/tarkbot_pure_pursuit/scripts/sac_actor_1.onnx"


def pure_pursuit_control(x, y, yaw, v, target_path, sl):
    tx, ty = target_path.X(sl), target_path.Y(sl)
    dx, dy = tx - x, ty - y
    alpha = math.atan2(dy, dx) - yaw
    LA = math.hypot(dx, dy)
    omega = 2 * abs(v) * math.sin(alpha) / LA
    return omega


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


def clipper(value, max_value, min_value):
    return max(min(value, max_value), min_value)
    

def angle_normalize(theta):
    return math.atan2(math.sin(theta), math.cos(theta))
    
    

if __name__ == '__main__':
    
    rospy.init_node('pure_pursuit_node', anonymous=True)
    rospy.Subscriber('/tarkbot_odom_raw', Odometry, odom_callback)
    cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    
    listener = tf.TransformListener()
    rate = rospy.Rate(20.0)
    rpath = ArcReferencePath("eight")
    
    ort_sess = ort.InferenceSession(onnx_path, providers=['AzureExecutionProvider', 'CPUExecutionProvider'])
   
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
        
        # nearest point
        sn = rpath.find_nearest_point(sn, x, y)
        
        # look-ahead point
        sl = sn + DEFAULT_L
        if last_sl < sl:
            last_sl = sl

        omega_ref = pure_pursuit_control(x, y, yaw, cv, rpath, last_sl)

        # current error
        cte, psi = rpath.calc_error(x, y, yaw, sn)
        
        # future error
        yaw_ref = rpath.calc_yaw(last_sl)
        psi2 = angle_normalize(yaw - yaw_ref)
        
        # concat and infer
        obs = np.array([[cte, psi, cv, comega, psi2]]).astype(np.float32)
        obs = np.clip(obs, min_val, max_val)
        action = ort_sess.run(None, {"input": obs})
        accel = 0.4 * np.tanh(action[0].item()) - 0.1
        v_ref = cv + accel * 0.05 * 2.2
        v_ref = np.clip(v_ref, 0.0, v_max)

        # velocity constrains
        v_ref = np.clip(v_ref, 0.0, v_max)
        omega_ref = np.clip(omega_ref, -omega_max, omega_max)
        
        cmd_vel = Twist()
        if rpath.len - sn > 0.02:
            cmd_vel.linear.x = v_ref
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

