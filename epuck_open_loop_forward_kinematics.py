# TASK 2:  Open Loop Forward Kinematics

import math
from epuck_helper_functions import steps_to_mm
from epuck_helper_functions import AXLE_LENGTH_MM


# Task 2: Forward Kinematics for Differential Drive Robot
def diff_drive_forward_kin(pose, left_steps, right_steps):
    """
    Compute the new pose of the robot after wheel movements using forward kinematics.

    Args:
        pose: Tuple (x, y, theta), current robot pose in mm and radians.
        left_steps: Number of steps moved by the left wheel.
        right_steps: Number of steps moved by the right wheel.

    Returns:
        Tuple (new_x, new_y, new_theta), new robot pose.
    """
    x, y, theta = pose

    # Convert steps to distances
    d_left = steps_to_mm(left_steps)
    d_right = steps_to_mm(right_steps)

    # Calculate linear and angular displacement
    delta_d = (d_left + d_right) / 2
    delta_theta = (d_right - d_left) / AXLE_LENGTH_MM

    # Update pose
    if abs(delta_theta) > 1e-6:  # Robot is turning
        radius = delta_d / delta_theta
        new_x = x + radius * (math.sin(theta + delta_theta) - math.sin(theta))
        new_y = y - radius * (math.cos(theta + delta_theta) - math.cos(theta))
    else:  # Robot is moving straight
        new_x = x + delta_d * math.cos(theta)
        new_y = y + delta_d * math.sin(theta)

    new_theta = (theta + delta_theta) % (2 * math.pi)  # Normalize theta

    return new_x, new_y, new_theta

# Example usage
if __name__ == "__main__":
    # Initial pose: (x=0 mm, y=0 mm, theta=0 radians)
    initial_pose = (0, 0, 0)

    # Test cases
    test_cases = [
        (0, 0),  # No movement
        (1290, 1290),  # Forward
        (-1290, -1290),  # Backward
        (1290, -1290),  # Rotate in place (clockwise)
        (2580, 0),  # Pivot around right wheel
    ]

    for left_steps, right_steps in test_cases:
        new_pose = diff_drive_forward_kin(initial_pose, left_steps, right_steps)
        print(f"Left steps: {left_steps}, Right steps: {right_steps}")
        print(f"New pose: x={new_pose[0]:.2f} mm, y={new_pose[1]:.2f} mm, theta={math.degrees(new_pose[2]):.2f}°\n")
