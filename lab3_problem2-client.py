#!/usr/bin/env python3

"""
Group Members: Krupal Shah & Jaspreet Singh Chhabra

Date: October 21th 2025
 
Brick Number: G20

Lab Number: 3

Problem Number: 2 (client side implementation)
 
Brief Program/Problem Description:
    -Implement Uncalibrated Visual Servoing (UVS) to move your 2-DOF planar robot arm to a desired position. 
        Your program should be able to dynamically track the end-effector of your robot and a moving target.

Brief Solution Summary:
    - Estimate the initial Jacobian. Tip: You can do that using orthogonal motions
    - Define a stop criteria for your looping algorithm
    - Implement Broyden update
    - Calculate Xdot base on the local linear model
    - Update X (motor joint angles)

Used Resources/Collaborators:
    - Lecture slides and notes from CMPUT 312.
    - Textbook: "Introduction to Robotics: Mechanics and Control" by John J. Craig.
    - OpenAI ChatGPT for code debugging assistance.

I/we hereby certify that I/we have produced the following solution 
using only the resources listed above in accordance with the 
CMPUT 312 collaboration policy.
"""

from VSMaterial import client
import utils as ut
import variables as v
from time import sleep


host = "192.168.0.3"
port = 9999
client = client.Client(host, port)
SPEED = 50

# reset motors
v.link_1_motor.reset()
v.link_2_motor.reset()


def main():
    while True:
        data = client.pollData()
        if len(data) > 0:
            if data == "EXIT":
                print("Termination signal received. Exiting...")
                break
            if data == "SAFETY_ON":
                v.SAFETY_MODE = True
                print("Safety mode enabled.")
                continue
            if data == "SAFETY_OFF":
                v.SAFETY_MODE = False
                print("Safety mode disabled.")
                continue
            if data.startswith("MOVE"):
                data = data.split(" ")
                angle1, angle2 = float(data[1]), float(data[2])
                ut.move_to_angles(SPEED, [angle1, angle2])
                client.sendDone()

        sleep(0.2)


if __name__ == "__main__":
    try:
        main()
        # handle keyboard interrupt
        v.link_1_motor.off()
        v.link_2_motor.off()
    except KeyboardInterrupt:
        v.link_1_motor.off()
        v.link_2_motor.off()
