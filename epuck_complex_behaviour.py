import time

import numpy as np

from epuck_helper_functions import steps_to_mm, mm_to_steps
from epuck_inverse_kinematics import diff_drive_inverse_kin
from epuck_ip import EPuckIP

# Task 1: Move the robot a specific number of motor steps
def move_steps(epuckcomm, l_speed_steps_s, r_speed_steps_s, l_target_steps, r_target_steps, Hz=10):
    """
    Move the robot based on motor steps.

    Args:
        epuckcomm: EPuck communication object (e.g., EPuckCom).
        l_speed_steps_s: Left wheel speed in steps per second.
        r_speed_steps_s: Right wheel speed in steps per second.
        l_target_steps: Target steps for the left wheel.
        r_target_steps: Target steps for the right wheel.
        Hz: Control loop frequency (default: 10 Hz).

    Returns:
        A tuple of actual steps moved: (left_steps_moved, right_steps_moved).
    """
    epuckcomm.state.act_left_motor_speed = l_speed_steps_s
    epuckcomm.state.act_right_motor_speed = r_speed_steps_s
    epuckcomm.state.sens_left_motor_steps = 0
    epuckcomm.state.sens_right_motor_steps = 0
    epuckcomm.send_command()
    epuckcomm.data_update()
    time.sleep(1 / Hz)
    left_start = epuckcomm.state.sens_left_motor_steps
    right_start = epuckcomm.state.sens_right_motor_steps
    left_moved = 0
    right_moved = 0
    i = 0

    while abs(left_moved) < abs(l_target_steps) or abs(right_moved) < abs(r_target_steps):
        epuckcomm.data_update()
        left_current = epuckcomm.state.sens_left_motor_steps
        right_current = epuckcomm.state.sens_right_motor_steps
        left_moved = left_current - left_start
        right_moved = right_current - right_start

        if abs(left_moved) >= abs(l_target_steps):
            epuckcomm.state.act_left_motor_speed = 0

        if abs(right_moved) >= abs(r_target_steps):
            epuckcomm.state.act_right_motor_speed = 0

        epuckcomm.send_command()
        i = i + 1
        print(left_moved, right_moved)
        time.sleep(1 / Hz)

    left_end = epuckcomm.state.sens_left_motor_steps
    right_end = epuckcomm.state.sens_right_motor_steps
    print(f"left end: {left_end}", f"right end: {right_end}")

    epuckcomm.stop_all()
    return left_moved, right_moved

# Task 1: Move the robot a specific distance in mm
def move_straight(epuckcomm, distance_mm, omega_rad,  Hz=10, mm_speed=100):
    """
    Move the robot a specific distance in mm.

    Args:
        epuckcomm: EPuck communication object (e.g., EPuckCom).
        distance_mm: Target distance in mm (negative for backward).
        Hz: Control loop frequency (default: 10 Hz).

    Returns:
        The actual distance moved based on odometry readings (in mm).
    """
    target_steps = mm_to_steps(distance_mm)
    speed_steps_s = int(mm_to_steps(mm_speed))  # Assume 100 mm/s speed

    left_moved, right_moved = move_steps(epuckcomm, (diff_drive_inverse_kin(distance_mm, mm_speed, omega_rad)), Hz)

    # Convert steps moved back to mm and return the average distance moved
    avg_steps = (left_moved + right_moved) / 2
    return steps_to_mm(avg_steps)

# Example usage
if __name__ == "__main__":
    from epuck_com import EPuckCom

    # epuck = EPuckCom("COM16", debug=True)
    epuck = EPuckIP("172.20.10.4", debug=True)
    epuck.enable_sensors = True

    if epuck.connect():
        print("Connected to e-puck!")

        # Move forward  500mm
        distance_moved = move_straight(epuck, 500, 0, mm_speed=100)
        print(f"Moved forward {distance_moved:.2f} mm")
        # turns 180 facing the starting point
        distance_moved = move_straight(epuck, 0, np.pi, mm_speed=100)
        print(f"Moved forward {distance_moved:.2f} mm")
        # moves forward 500mm towards the starting point
        distance_moved = move_straight(epuck, 500, 0, mm_speed=100)
        print(f"Moved forward {distance_moved:.2f} mm")
        # turns 180 facing away.
        distance_moved = move_straight(epuck, 0, np.pi, mm_speed=100)
        print(f"Moved forward {distance_moved:.2f} mm")



        epuck.close()
    else:
        print("Failed to connect to e-puck.")








