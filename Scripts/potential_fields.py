import numpy as np
from geometry_msgs.msg import Twist

def laser_to_cartesian(scan):
    angles = np.linspace(scan.angle_min, scan.angle_max, len(scan.ranges))
    ranges = np.array(scan.ranges)

    xs = ranges * np.cos(angles)
    ys = ranges * np.sin(angles)

    mask = np.isfinite(xs) & np.isfinite(ys) & (ranges > 0.05) & (ranges < 2.5)
    return np.vstack((xs[mask], ys[mask])).T

def compute_repulsive_force(points):
    force = np.zeros(2)

    for p in points:
        dist = np.linalg.norm(p)

        # Ignore obstacles behind the robot
        if p[0] < 0:
            continue

        if dist < 0.6:
            repulse = -p / (dist**2 + 1e-6)
            force += repulse

    return 0.3 * force  # Reduced gain

def force_to_cmd(force_vec):
    cmd = Twist()
    angle = np.arctan2(force_vec[1], force_vec[0])
    magnitude = np.linalg.norm(force_vec)

    if magnitude < 0.05:
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        return cmd

    cmd.linear.x = min(0.25, magnitude)
    
    # Prevent tight spirals — limit rotation influence
    cmd.angular.z = 1.5 * angle
    cmd.angular.z = np.clip(cmd.angular.z, -0.8, 0.8)

    return cmd

