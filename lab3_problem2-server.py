#!/usr/bin/env python3

# from inv_kine_newtons import line_waypoints
import numpy as np
from queue import Queue
from time import sleep
from VSMaterial import server
from VSMaterial import color_tracking
import math

host = "192.168.0.3"
port = 9999
server = server.Server(host, port)
queue = Queue()
SPEED = 30
tracker = color_tracking.Tracker('b', 'g')


def send_angles(theta1, theta2):
    server.sendAngles(theta1, theta2, queue)
    while True:
        if queue.get() == "DONE":
            break
        sleep(0.1)
    sleep(0.4)


def calulate_jacobian():
    curr_pos, _ = get_image_position()

    # Joint 1
    send_angles(10, 0)
    position_1, _ = get_image_position()
    delta_u = (position_1[0] - curr_pos[0], position_1[1] - curr_pos[1])
    send_angles(0, 0)

    # Joint 2
    send_angles(0, -10)
    position_2, _ = get_image_position()
    delta_v = (position_2[0] - curr_pos[0], position_2[1] - curr_pos[1])
    send_angles(0, 0)

    J = [[delta_u[0]/10, delta_v[0]/10],
         [delta_u[1]/10, delta_v[1]/10]]
    J = np.array(J)

    return J


def inv_kin_broyden(
        current_pos: (list | tuple),
        target_pos: (list | tuple),
        initial_jacobian: np.ndarray,
        alpha=0.5,
        max_iters=10, threshold=30):
    """
    Inverse kinematics using Broyden's method to update the Jacobian.

    Parameters:
    - current_pos: Current position of the end-effector (x, y).
    - target_pos: Desired position of the end-effector (x, y).
    - initial_jacobian: Initial guess for the Jacobian matrix.
    - alpha: Step size for the update.
    - max_iters: Maximum number of iterations.
    - threshold: Convergence threshold for the position error.

    Returns:
    - joint_angles: Estimated joint angles to reach the target position.
    """
    joint_angles = [5.0, -5.0]  # Initial guess for joint angles
    jacobian = initial_jacobian
    waypoints = line_waypoints(current_pos, target_pos, max_step=20)

    for waypoint in waypoints:
        target_pos = waypoint
        for _ in range(max_iters):
            # Get current end-effector position from image
            current_pos, _ = get_image_position()

            # Compute error
            error = (target_pos[0] - current_pos[0],
                     target_pos[1] - current_pos[1])
            error_norm = np.linalg.norm(error)

            if error_norm < threshold:
                print(f"threshold met: (error={error_norm:.2f})")
                
                break

            # Compute inverse of Jacobian (2x2 matrix inversion)
            det = np.linalg.det(jacobian)
            if abs(det) < 1e-6:
                print("Jacobian is singular, recalculating...")
                jacobian = calulate_jacobian()
                det = np.linalg.det(jacobian)

            inv_jacobian = np.linalg.pinv(jacobian)

            # Update joint angles using the inverse Jacobian
            delta_theta = alpha * np.dot(inv_jacobian, np.array(error))
            max_step_deg = 10.0
            max_mag = max(abs(delta_theta[0]), abs(delta_theta[1]))
            if max_mag > max_step_deg:
                scale = max_step_deg / max_mag
                delta_theta[0] *= scale
                delta_theta[1] *= scale
                
            joint_angles[0] += delta_theta[0]
            joint_angles[1] += delta_theta[1]

            # Send angles to robot
            send_angles(joint_angles[0], joint_angles[1])

            # Get new position after movement
            new_pos, _ = get_image_position()
            delta_pos = (new_pos[0] - current_pos[0],
                         new_pos[1] - current_pos[1])

            # Update Jacobian using Broyden's method
            if abs(delta_theta[0]) > 1e-6 or abs(delta_theta[1]) > 1e-6:
                delta_theta_vec = np.array(
                    [[delta_theta[0]], [delta_theta[1]]])
                delta_pos_vec = np.array([[delta_pos[0]], [delta_pos[1]]])
                jacobian += (((delta_pos_vec - np.dot(jacobian, delta_theta_vec)).dot(
                    delta_theta_vec.T)) / (delta_theta_vec.T.dot(delta_theta_vec))) * alpha


def get_image_position():
    while True:
        point = np.array(tracker.point)
        goal = np.array(tracker.goal)
        if np.all(point != 0) and np.all(goal != 0):
            sleep(0.1)
            print("Current Point:", tracker.point, "Goal Point:", tracker.goal)
            return tracker.point[0][:2], tracker.goal[0][:2]

def line_waypoints(start, goal, max_step=20.0):  # mm per step
    dx, dy = goal[0]-start[0], goal[1]-start[1]
    dist = math.hypot(dx, dy)
    if dist <= max_step:
        return [goal]
    n = int(math.ceil(dist/max_step))
    return [(start[0] + (i/n)*dx, start[1] + (i/n)*dy) for i in range(1, n+1)]

def main():
    current_point, goal_point = get_image_position()
    print("Rover is at:", current_point, "Goal is at:", goal_point)
    sleep(1)

    init_jac = calulate_jacobian()
    print("Initial Jacobian:", init_jac)

    inv_kin_broyden(current_point, goal_point, init_jac)


if __name__ == "__main__":
    main()
