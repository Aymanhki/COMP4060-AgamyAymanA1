import math
import numpy as np
import epuck_helper_functions as helper

def diff_drive_inverse_kin(distance_mm, speed_mm_s, omega_rad):
    """
    :param distance_mm: distance to be travelled
    :param speed_mm_s: signed speed, negative if moving backwards
    :param omega_rad: angle of turn
    :return: left wheel speed, right wheel speed (in steps), total_left_steps, total_right_steps

    """
    if distance_mm == 0:
        # Angular velocity (rad/s) derived from speed and axle length
        angular_velocity_rad = abs(speed_mm_s) / (helper.AXLE_LENGTH_MM / 2)

        # Distances each wheel travels during the rotation
        left_distance_mm = -omega_rad * helper.AXLE_LENGTH_MM / 2
        right_distance_mm = omega_rad * helper.AXLE_LENGTH_MM / 2

        # Wheel speeds (mm/s)
        left_speed_mm = -angular_velocity_rad * (helper.AXLE_LENGTH_MM / 2)
        right_speed_mm = angular_velocity_rad * (helper.AXLE_LENGTH_MM / 2)

    else:
        time_s = abs(distance_mm / speed_mm_s)  # Time to travel the given distance
        angular_velocity_rad = omega_rad / time_s if time_s != 0 else 0

        left_speed_mm = speed_mm_s - (angular_velocity_rad * helper.AXLE_LENGTH_MM / 2)
        right_speed_mm = speed_mm_s + (angular_velocity_rad * helper.AXLE_LENGTH_MM / 2)

        left_distance_mm = distance_mm - (omega_rad * helper.AXLE_LENGTH_MM / 2)
        right_distance_mm = distance_mm + (omega_rad * helper.AXLE_LENGTH_MM / 2)

    # Convert speeds to steps/s
    left_speed_steps = helper.mm_to_steps(left_speed_mm)
    right_speed_steps = helper.mm_to_steps(right_speed_mm)

    # Convert distances to total steps
    left_steps = abs(helper.mm_to_steps(left_distance_mm))
    right_steps = abs(helper.mm_to_steps(right_distance_mm))

    return left_speed_steps, right_speed_steps, left_steps, right_steps

if __name__ == "__main__":
    test_cases = [
        (130, 10, 0, (75, 75, 978, 978)),
        (130, -10, 0, (-75, -75, 978, 978)),
        (300, 50, 0, (376, 376, 2257, 2257)),
        (200, 70, np.pi / 4, (472, 582, 1348, 1661)),
        (-200, 70, np.pi / 4, (472, 582, 1348, 1661)),
        (300, -40, -np.pi * 2, (-134, -468, 1348, 1661)),
        (0, 100, -np.pi * 2, (753, -753, 1253, -1253)),
        (0, 50, np.pi / 2, (-376, 376, -313, 313)),
        (0, -50, np.pi / 2, (-376, 376, -313, 313))
    ]

    for idx, (distance, speed, omega, expected) in enumerate(test_cases, 1):
        actual = diff_drive_inverse_kin(distance, speed, omega)
        print(f"Test Case #{idx}:")
        print(f"Expected: {expected}")
        print(f"Actual:   {actual}\n")
