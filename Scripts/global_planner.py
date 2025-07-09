#!/usr/bin/env python3

import rospy
from nav_msgs.msg import OccupancyGrid
import numpy as np
import heapq
import os

def a_star(grid, start, goal):
    h, w = grid.shape
    open_set = []
    heapq.heappush(open_set, (0, start))
    came_from = {}
    g_score = {start: 0}

    def heuristic(a, b):
        return abs(a[0] - b[0]) + abs(a[1] - b[1])

    while open_set:
        _, current = heapq.heappop(open_set)
        if current == goal:
            break

        for dx, dy in [(-1,0), (1,0), (0,-1), (0,1)]:
            neighbor = (current[0]+dx, current[1]+dy)
            if 0 <= neighbor[0] < h and 0 <= neighbor[1] < w:
                if grid[neighbor] > 50:
                    continue
                tentative_g = g_score[current] + 1
                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    g_score[neighbor] = tentative_g
                    f = tentative_g + heuristic(neighbor, goal)
                    heapq.heappush(open_set, (f, neighbor))
                    came_from[neighbor] = current

    path = []
    node = goal
    while node != start:
        path.append(node)
        node = came_from.get(node)
        if node is None:
            print(" No path found.")
            return []
    path.append(start)
    path.reverse()
    return path

def main():
    rospy.init_node('global_planner_with_obstacle')

    print(" Waiting for map...")
    msg = rospy.wait_for_message('/map', OccupancyGrid)
    print("Map received")

    width = msg.info.width
    height = msg.info.height
    resolution = msg.info.resolution
    origin = msg.info.origin
    grid = np.array(msg.data).reshape((height, width))

    #  Set better start and goal in world coordinates
    start_world = (-2.0, -0.5)
    goal_world = (2.0, 0.5)

    
    start = (
        int((start_world[1] - origin.position.y) / resolution),
        int((start_world[0] - origin.position.x) / resolution)
    )
    goal = (
        int((goal_world[1] - origin.position.y) / resolution),
        int((goal_world[0] - origin.position.x) / resolution)
    )

    path = a_star(grid, start, goal)
    if not path:
        return

    real_path = []
    for row, col in path:
        x = col * resolution + origin.position.x
        y = row * resolution + origin.position.y
        real_path.append((x, y))

    save_path = "/home/va3803/catkin_ws/src/turtlebot3_nav_student/scripts/waypoints.npy"
    try:
        np.save(save_path, real_path)
        print(f" Waypoints saved to {save_path}")
    except Exception as e:
        print(f" Failed to save file: {e}")

if __name__ == '__main__':
    main()

