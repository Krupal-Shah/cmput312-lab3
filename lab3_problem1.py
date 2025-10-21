#!/usr/bin/env python3

"""
Group Members: Krupal Shah & Jaspreet Singh Chhabra

Date: October 21th 2025
 
Brick Number: G20

Lab Number: 3

Problem Number: 1
 
Brief Program/Problem Description:
    - Add the following functionalities to your 2DOF robot arm:
        Draw a straight line defined by two points.

Brief Solution Summary:
    - Implemented inverse kinematics using lecture slides.
    - Reused methods from lab2_problem3 for inverse kinematics.
    - Created a function to generate waypoints along a straight line between two points.

Used Resources/Collaborators:
    - Lecture slides and notes from CMPUT 312.
    - Textbook: "Introduction to Robotics: Mechanics and Control" by John J. Craig.

I/we hereby certify that I/we have produced the following solution 
using only the resources listed above in accordance with the 
CMPUT 312 collaboration policy.
"""

from inv_kine_newtons import line_waypoints
from lab2_problem3 import invKin2D
from time import sleep
import variables as v


def main():
    inv_mode = 1  # 0 for Newton's method, 1 for Analytical method
    start_point = (15.0, 4.0)  # Starting point (x1, y1)
    end_point = (12.0, -5.0)    # Ending point (x2, y2)
    v.link_1_motor.reset()
    v.link_2_motor.reset()

    # Move to start point:
    _ = invKin2D(start_point, inv_mode)
    print("Moved to start point")
    sleep(2)

    # Move to the start point
    waypoints = line_waypoints(start_point, end_point, max_step=0.3)
    print("Waypoints:", waypoints)
    for waypoint in waypoints:
        _ = invKin2D(waypoint, inv_mode)
        sleep(0.5)  # Small delay between movements


if __name__ == "__main__":
    main()
