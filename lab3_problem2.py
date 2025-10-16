from VSMaterial import *
from time import sleep
from queue import Queue

tracker = color_tracking.Tracker('b', 'g')
host = "192.168.0.2"
port = 9999
server = server.Server(host, port)
queue = Queue()


def calibrate_jacobian():
    curr_pos = tracker.point

    server.sendAngles(10, 0, queue)
    sleep(2)
    pos1 = tracker.point
    delta_u = (pos1[0] - curr_pos[0], pos1[1] - curr_pos[1])

    server.sendAngles(0, 10, queue)
    sleep(2)
    pos2 = tracker.point
    delta_v = (pos2[0] - curr_pos[0], pos2[1] - curr_pos[1])

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
        # Compute current end-effector position based on joint angles
        x = (joint_angles[0] + joint_angles[1]) * \
            10  # Simplified forward kinematics
        y = (joint_angles[0] + joint_angles[1]) * 10
        current_pos = (x, y)

        # Compute error
        error = (target_pos[0] - current_pos[0],
                 target_pos[1] - current_pos[1])
        if abs(error[0]) < 1e-3 and abs(error[1]) < 1e-3:
            break  # Converged

        # Update joint angles using the Jacobian
        delta_theta = [alpha * (jacobian[0][0] * error[0] + jacobian[0][1] * error[1]),
                       alpha * (jacobian[1][0] * error[0] + jacobian[1][1] * error[1])]
        joint_angles[0] += delta_theta[0]
        joint_angles[1] += delta_theta[1]

        # Update Jacobian using Broyden's method
        new_x = (joint_angles[0] + joint_angles[1]) * 10
        new_y = (joint_angles[0] + joint_angles[1]) * 10
        new_pos = (new_x, new_y)
        delta_pos = (new_pos[0] - current_pos[0], new_pos[1] - current_pos[1])

        if delta_pos != (0, 0):
            jacobian_update = [[(delta_theta[0] * delta_pos[0]) / (delta_pos[0]**2 + delta_pos[1]**2),
                                (delta_theta[0] * delta_pos[1]) / (delta_pos[0]**2 + delta_pos[1]**2)],
                               [(delta_theta[1] * delta_pos[0]) / (delta_pos[0]**2 + delta_pos[1]**2),
                                (delta_theta[1] * delta_pos[1]) / (delta_pos[0]**2 + delta_pos[1]**2)]]
            jacobian[0][0] += jacobian_update[0][0]
            jacobian[0][1] += jacobian_update[0][1]
            jacobian[1][0] += jacobian_update[1][0]
            jacobian[1][1] += jacobian_update[1][1]

    return joint_angles


def main():
    while True:
        if tracker.point is not None and tracker.goal is not None:
            current_point = tracker.point
            goal_point = tracker.goal
        break

    print("Point is at: "+str(current_point), "Goal is at: "+str(goal_point))
    sleep(2)

    init_jac = calibrate_jacobian()
    inv_kin_broyden(current_point, goal_point, init_jac)
    sleep(2)


if __name__ == "__main__":
    main()
