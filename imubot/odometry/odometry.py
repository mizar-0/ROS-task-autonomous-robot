#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion
import numpy as np
import tf.transformations as transformations

# Global variables
x = 0.0
y = 0.0
theta = 0.0
vx = 0.0
vy = 0.0
last_time = None

def imu_callback(msg):
    global x, y, vx, vy, last_time

    current_time = rospy.Time.now()
    if last_time is None:
        last_time = current_time
        return

    dt = (current_time - last_time).to_sec()

    # Extract linear acceleration and orientation
    ax = msg.linear_acceleration.x
    ay = msg.linear_acceleration.y
    az = msg.linear_acceleration.z

    # Get orientation quaternion from IMU
    q = msg.orientation
    imu_quat = [q.x, q.y, q.z, q.w]

    # Transform acceleration from IMU frame to the world frame
    acc_world = transform_acceleration_to_world_frame(ax, ay, az, imu_quat)

    # Integrate acceleration to get velocity
    vx += acc_world[0] * dt
    vy += acc_world[1] * dt

    # Integrate velocity to get position
    x += vx * dt
    y += vy * dt

    # Create and publish the odometry message
    odom = Odometry()
    odom.header.stamp = current_time
    odom.header.frame_id = 'odom'
    odom.child_frame_id = 'base_link'

    odom.pose.pose.position.x = x
    odom.pose.pose.position.y = y
    odom.pose.pose.position.z = 0.0

    #(*) here unpacks the list so we dont have to use indices to specify each item
    odom.pose.pose.orientation = Quaternion(*imu_quat)

    odom.twist.twist.linear.x = vx
    odom.twist.twist.linear.y = vy
    odom.twist.twist.linear.z = 0.0
    odom.twist.twist.angular.z = msg.angular_velocity.z

    odom_pub.publish(odom)
    last_time = current_time

def transform_acceleration_to_world_frame(ax, ay, az, imu_quat):
    # Create a vector for the acceleration
    acc_vector = np.array([ax, ay, az, 0.0])

    # Convert quaternion to rotation matrix
    rotation_matrix = quaternion_to_rotation_matrix(imu_quat)

    # Rotate the acceleration vector into the world frame
    acc_world = np.dot(rotation_matrix, acc_vector)[:3]

    return acc_world

def quaternion_to_rotation_matrix(quat):
    # Convert quaternion to rotation matrix
    return transformations.quaternion_matrix(quat)


if __name__ == '__main__':
    rospy.init_node('imu_odometry_node')

    # Publisher for odometry
    odom_pub = rospy.Publisher('odometry', Odometry, queue_size=50)

    # Subscriber for IMU data
    rospy.Subscriber('imu', Imu, imu_callback)

    # rate = rospy.Rate(100)  # 10 Hz
    # while not rospy.is_shutdown():
    #     rate.sleep()

    rospy.spin()