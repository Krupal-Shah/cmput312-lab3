#!/usr/bin/env python3

from VSMaterial import *
import utils as ut
import variables as v
from time import sleep
from queue import Queue


host = "192.168.0.2"
port = 9999
server = server.Server(host, port)
queue = Queue()
SPEED = 50
tracker = color_tracking.Tracker()


def calulate_jacobian():
    curr_pos, _ = get_image_position()

    server.sendAngles(10, 0, queue)
    while True:
        if queue.get() == "DONE":
            break
        sleep(0.1)
    position_1, _ = get_image_position()
    delta_u = (position_1[0] - curr_pos[0], position_1[1] - curr_pos[1])

    server.sendAngles(10, 0, queue)
    while True:
        if queue.get() == "DONE":
            break
        sleep(0.1)
    position_2, _ = get_image_position()
    delta_v = (position_2[0] - curr_pos[0], position_2[1] - curr_pos[1])

    J = [[delta_u[0]/10, delta_v[0]/10],
         [delta_u[1]/10, delta_v[1]/10]]

    return J


def inv_kin_broyden(current_pos, target_pos, initial_jacobian, alpha=0.1, max_iters=100):
    """
    Inverse kinematics using Broyden's method to update the Jacobian.

    Parameters:
    - current_pos: Current position of the end-effector (x, y).
    - target_pos: Desired position of the end-effector (x, y).
    - initial_jacobian: Initial guess for the Jacobian matrix.
    - alpha: Step size for the update.
    - max_iters: Maximum number of iterations.

    Returns:
    - joint_angles: Estimated joint angles to reach the target position.
    """
    joint_angles = [0.0, 0.0]  # Initial guess for joint angles
    jacobian = initial_jacobian

    for _ in range(max_iters):
        # Get current end-effector position from image
        current_pos, _ = get_image_position()

        # Compute error
        error = (target_pos[0] - current_pos[0],
                 target_pos[1] - current_pos[1])
        if abs(error[0]) < 1e-3 and abs(error[1]) < 1e-3:
            break  # Converged

        # Compute inverse of Jacobian (2x2 matrix inversion)
        det = jacobian[0][0] * jacobian[1][1] - jacobian[0][1] * jacobian[1][0]
        if abs(det) < 1e-6:
            print("Jacobian is singular, recalculating...")
            jacobian = calulate_jacobian()
            det = jacobian[0][0] * jacobian[1][1] - \
                jacobian[0][1] * jacobian[1][0]

        inv_jacobian = [[jacobian[1][1]/det, -jacobian[0][1]/det],
                        [-jacobian[1][0]/det, jacobian[0][0]/det]]

        # Update joint angles using the inverse Jacobian
        delta_theta = [alpha * (inv_jacobian[0][0] * error[0] + inv_jacobian[0][1] * error[1]),
                       alpha * (inv_jacobian[1][0] * error[0] + inv_jacobian[1][1] * error[1])]
        joint_angles[0] += delta_theta[0]
        joint_angles[1] += delta_theta[1]

        # Send angles to robot
        server.sendAngles(delta_theta[0], delta_theta[1], queue)
        while True:
            if queue.get() == "DONE":
                break
            sleep(0.1)

        # Get new position after movement
        new_pos, _ = get_image_position()
        delta_pos = (new_pos[0] - current_pos[0], new_pos[1] - current_pos[1])

        # Update Jacobian using Broyden's method
        if abs(delta_theta[0]) > 1e-6 or abs(delta_theta[1]) > 1e-6:
            delta_theta_norm_sq = delta_theta[0]**2 + delta_theta[1]**2
            broyden_term = [[(delta_pos[0] - jacobian[0][0]*delta_theta[0] - jacobian[0][1]*delta_theta[1]) * delta_theta[0] / delta_theta_norm_sq,
                             (delta_pos[0] - jacobian[0][0]*delta_theta[0] - jacobian[0][1]*delta_theta[1]) * delta_theta[1] / delta_theta_norm_sq],
                            [(delta_pos[1] - jacobian[1][0]*delta_theta[0] - jacobian[1][1]*delta_theta[1]) * delta_theta[0] / delta_theta_norm_sq,
                             (delta_pos[1] - jacobian[1][0]*delta_theta[0] - jacobian[1][1]*delta_theta[1]) * delta_theta[1] / delta_theta_norm_sq]]

            jacobian[0][0] += broyden_term[0][0]
            jacobian[0][1] += broyden_term[0][1]
            jacobian[1][0] += broyden_term[1][0]
            jacobian[1][1] += broyden_term[1][1]

    return joint_angles


def get_image_position():
    while True:
        if tracker.point == (0, 0, 0) or tracker.goal == (0, 0, 0):
            sleep(1)
            break

    return (tracker.point[0], tracker.point[1]), (tracker.goal[0], tracker.goal[1])


def main():
    current_point, goal_point = get_image_position()
    print("Rover is at: "+str(current_point), "Goal is at: "+str(goal_point))
    sleep(1)

    init_jac = calulate_jacobian()
    inv_kin_broyden(current_point, goal_point, init_jac)
    sleep(2)


if __name__ == "__main__":
    main()
