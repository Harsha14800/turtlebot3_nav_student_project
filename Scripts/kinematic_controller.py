#!/usr/bin/env python3

import rospy
import numpy as np
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import tf
import math

waypoints = [(0.5, 0.0), (1.0, 0.0), (1.0, 0.5), (1.0, 1.0)]  # Replace with real-world path
current_waypoint_idx = 0

def odom_callback(msg):
    global current_waypoint_idx

    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y

    orientation_q = msg.pose.pose.orientation
    (_, _, theta) = tf.transformations.euler_from_quaternion([
        orientation_q.x,
        orientation_q.y,
        orientation_q.z,
        orientation_q.w
    ])

    if current_waypoint_idx >= len(waypoints):
        cmd = Twist()
        cmd_pub.publish(cmd)
        return

    goal_x, goal_y = waypoints[current_waypoint_idx]

    dx = goal_x - x
    dy = goal_y - y
    rho = math.sqrt(dx**2 + dy**2)
    alpha = math.atan2(dy, dx) - theta
    beta = -theta - alpha

    v = 0.5 * rho
    w = 1.0 * alpha + 0.0 * beta

    cmd = Twist()
    cmd.linear.x = min(v, 0.3)
    cmd.angular.z = w
    cmd_pub.publish(cmd)

    if rho < 0.1:
        current_waypoint_idx += 1
        rospy.loginfo(f"Reached waypoint {current_waypoint_idx}")

def main():
    global cmd_pub
    rospy.init_node('kinematic_controller')
    cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rospy.Subscriber('/odom', Odometry, odom_callback)
    rospy.spin()

if __name__ == '__main__':
    main()

