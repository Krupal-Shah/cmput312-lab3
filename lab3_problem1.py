#!/usr/bin/env python3

from inv_kine_newtons import line_waypoints
from lab2_problem3 import invKin2D
from time import sleep


def main():
    inv_mode = 1  # 0 for Newton's method, 1 for Analytical method
    start_point = (10.0, 5.0)  # Starting point (x1, y1)
    end_point = (10.0, -5.0)    # Ending point (x2, y2)

    # Move to start point:
    _ = invKin2D(start_point, inv_mode)
    print("Moved to start point")
    sleep(2)

    # Move to the start point
    waypoints = line_waypoints(start_point, end_point, max_step=0.3)
    print("Waypoints:", waypoints)
    for waypoint in waypoints:
        _ = invKin2D(waypoint, inv_mode)


if __name__ == "__main__":
    main()
