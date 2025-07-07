#!/usr/bin/env python3

import rospy
import time
import math
import numpy as np

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion

import sys
import os
sys.path.append(os.path.dirname(__file__))

from potential_fields import laser_to_cartesian, compute_repulsive_force, force_to_cmd

# === Global variables ===
waypoints = []
current_pose = None
scan_data = None
log_data = []

goal_tolerance = 0.5
obstacle_threshold = 0.4
avoidance_count = 0
recovery_count = 0

# === Load waypoints from file ===
def load_waypoints():
    global waypoints
    waypoints = np.load("/home/va3803/catkin_ws/src/turtlebot3_nav_student/scripts/waypoints.npy")
    print("✅ Loaded waypoints:", waypoints.shape)

# === Odometry callback ===
def odom_callback(msg):
    global current_pose
    pos = msg.pose.pose.position
    ori = msg.pose.pose.orientation
    _, _, yaw = euler_from_quaternion([ori.x, ori.y, ori.z, ori.w])
    current_pose = (pos.x, pos.y, yaw)

# === LIDAR callback ===
def scan_callback(msg):
    global scan_data
    scan_data = msg

# === Distance between two (x, y) points ===
def distance(p1, p2):
    return np.linalg.norm(np.array(p1[:2]) - np.array(p2[:2]))

# === Compute attractive force ===
def compute_attractive_force(goal_x, goal_y):
    x, y, _ = current_pose
    force = np.array([goal_x - x, goal_y - y])
    norm = np.linalg.norm(force)
    if norm > 0:
        return 0.6 * force / norm
    return np.zeros(2)

# === Main navigation loop ===
def main():
    global current_pose, scan_data, log_data, avoidance_count, recovery_count

    rospy.init_node("navigator")
    cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
    rospy.Subscriber("/odom", Odometry, odom_callback)
    rospy.Subscriber("/scan", LaserScan, scan_callback)

    load_waypoints()
    rate = rospy.Rate(10)
    current_index = 0

    rospy.sleep(2)

    wall_start = time.time()
    ros_start = rospy.Time.now()
    max_duration = 300

    stuck_counter = 0
    last_index = -1
    last_force_dir = np.array([1.0, 0.0])

    total_distance_traveled = 0.0
    previous_pose = None

    while not rospy.is_shutdown() and current_index < len(waypoints):
        if time.time() - wall_start > max_duration:
            print("⏰ Timeout reached.")
            break

        if current_pose is None or scan_data is None:
            rate.sleep()
            continue

        x, y, _ = current_pose
        goal_x, goal_y = waypoints[current_index]
        dist_to_goal = distance(current_pose, (goal_x, goal_y))
        obstacle = min(scan_data.ranges) < obstacle_threshold

        if previous_pose is not None:
            total_distance_traveled += distance(current_pose, previous_pose)
        previous_pose = current_pose

        if obstacle:
            points = laser_to_cartesian(scan_data)
            repulsive = compute_repulsive_force(points)
            attractive = compute_attractive_force(goal_x, goal_y)
            total_force = 0.9 * attractive + 0.6 * repulsive
            total_force = 0.7 * last_force_dir + 0.3 * total_force
            norm = np.linalg.norm(total_force)
            if norm > 0:
                total_force = total_force / norm
            last_force_dir = total_force
            cmd = force_to_cmd(total_force)
            avoidance_count += 1
            print(f"⚠️ Obstacle detected — combined field at wp {current_index}")
        else:
            attractive = compute_attractive_force(goal_x, goal_y)
            cmd = force_to_cmd(attractive)
            print(f"🚗 Moving to waypoint {current_index}: ({goal_x:.2f}, {goal_y:.2f})")

        # Log per-step data
        log_data.append((
            time.time(), x, y, goal_x, goal_y, dist_to_goal,
            obstacle, cmd.linear.x, cmd.angular.z
        ))

        cmd_pub.publish(cmd)

        if dist_to_goal < goal_tolerance:
            print(f"✅ Reached waypoint {current_index}")
            current_index += 1
            stuck_counter = 0
            last_index = current_index
        else:
            if current_index == last_index:
                stuck_counter += 1
            else:
                stuck_counter = 0
                last_index = current_index

        if stuck_counter > 40:
            print("🛑 Stuck detected — escape maneuver")
            recovery_count += 1

            twist = Twist()
            twist.linear.x = -0.1
            for _ in range(10):
                cmd_pub.publish(twist)
                rate.sleep()

            twist = Twist()
            direction = np.random.choice([-1, 1])
            twist.angular.z = direction * 1.0
            for _ in range(15):
                cmd_pub.publish(twist)
                rate.sleep()

            stuck_counter = 0

        rate.sleep()

    wall_end = time.time()
    ros_end = rospy.Time.now()
    total_wall_time = wall_end - wall_start
    total_ros_time = (ros_end - ros_start).to_sec()

    cmd_pub.publish(Twist())
    rospy.sleep(1)

    wps = waypoints
    planned_path_length = sum(np.linalg.norm(wps[i+1] - wps[i]) for i in range(len(wps)-1))

    final_dist_to_goal = distance(current_pose, waypoints[-1])
    avg_speed = total_distance_traveled / total_wall_time if total_wall_time > 0 else 0.0

    metrics_path = "/home/va3803/catkin_ws/src/turtlebot3_nav_student/scripts/navigation_metrics.txt"
    with open(metrics_path, "w") as f:
        f.write(f"Wall Time: {total_wall_time:.2f} sec\n")
        f.write(f"ROS Time: {total_ros_time:.2f} sec\n")
        f.write(f"Planned Path Length: {planned_path_length:.2f} meters\n")
        f.write(f"Actual Distance Traveled: {total_distance_traveled:.2f} meters\n")
        f.write(f"Avoidance Count: {avoidance_count}\n")
        f.write(f"Recovery Maneuvers: {recovery_count}\n")
        f.write(f"Waypoints: {len(waypoints)}\n")
        f.write(f"Final Distance to Goal: {final_dist_to_goal:.2f} m\n")
        f.write(f"Average Speed: {avg_speed:.3f} m/s\n")

    log_path = "/home/va3803/catkin_ws/src/turtlebot3_nav_student/scripts/log_run.txt"
    with open(log_path, "w") as f:
        f.write("Time,X,Y,Goal_X,Goal_Y,Distance_to_Goal,Obstacle,Linear_Vel,Angular_Vel\n")
        for t, x, y, gx, gy, dist, obs, lin, ang in log_data:
            f.write(f"{t:.2f},{x:.3f},{y:.3f},{gx:.3f},{gy:.3f},{dist:.3f},{int(obs)},{lin:.3f},{ang:.3f}\n")

    print("🎉 Navigation completed.")
    print(f"🕒 ROS Time: {total_ros_time:.2f} sec | ⏱️ Wall Time: {total_wall_time:.2f} sec")
    print(f"📏 Actual Distance: {total_distance_traveled:.2f} m | Planned Path: {planned_path_length:.2f} m")
    print(f"🚧 Avoidances: {avoidance_count} | 🛠️ Recoveries: {recovery_count}")
    print(f"🧭 Final Goal Distance: {final_dist_to_goal:.2f} m | 🚀 Avg Speed: {avg_speed:.2f} m/s")

if __name__ == "__main__":
    main()

