# TASK 1: Simple Open Loop Controller

import time
from epuck_helper_functions import steps_to_mm, mm_to_steps
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
    # Set the target motor speeds
    epuckcomm.state.act_left_motor_speed = l_speed_steps_s
    epuckcomm.state.act_right_motor_speed = r_speed_steps_s
    epuckcomm.send_command()

    # Wait briefly to allow updates
    time.sleep(1 / Hz)
    epuckcomm.data_update()

    # Record the initial motor step counts
    left_start = epuckcomm.state.sens_left_motor_steps
    right_start = epuckcomm.state.sens_right_motor_steps

    left_moved = 0
    right_moved = 0

    while abs(left_moved) < abs(l_target_steps) or abs(right_moved) < abs(r_target_steps):
        epuckcomm.data_update()
        left_current = epuckcomm.state.sens_left_motor_steps
        right_current = epuckcomm.state.sens_right_motor_steps

        # Calculate the relative steps moved
        left_moved = left_current - left_start
        right_moved = right_current - right_start

        # Stop the left motor if the target is reached
        if abs(left_moved) >= abs(l_target_steps):
            epuckcomm.state.act_left_motor_speed = 0

        # Stop the right motor if the target is reached
        if abs(right_moved) >= abs(r_target_steps):
            epuckcomm.state.act_right_motor_speed = 0

        # Send updated commands to the robot
        epuckcomm.send_command()
        print(f"Left Moved: {left_moved}, Right Moved: {right_moved}")
        time.sleep(1 / Hz)

    # Final motor step counts
    left_end = epuckcomm.state.sens_left_motor_steps
    right_end = epuckcomm.state.sens_right_motor_steps
    print(f"Final Left: {left_end}, Final Right: {right_end}")

    # Stop all motors
    epuckcomm.stop_all()
    return left_moved, right_moved

# Task 1: Move the robot a specific distance in mm
def move_straight(epuckcomm, distance_mm, Hz=10, mm_speed=100):
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

    left_moved, right_moved = move_steps(epuckcomm, speed_steps_s, speed_steps_s, target_steps, target_steps, Hz)

    # Convert steps moved back to mm and return the average distance moved
    avg_steps = (left_moved + right_moved) / 2
    return steps_to_mm(avg_steps)

# Example usage
if __name__ == "__main__":
    from epuck_com import EPuckCom

    # epuck = EPuckCom("COM16", debug=True)
    epuck = EPuckIP("192.168.1.19", debug=True)
    epuck.enable_sensors = True

    if epuck.connect():
        print("Connected to e-puck!")

        # Example: Move forward 100 mm
        distance_moved = move_straight(epuck, 130, mm_speed=100)
        print(f"Moved forward {distance_moved:.2f} mm")

        # Example: Move backward 50 mm
        distance_moved = move_straight(epuck, -130, mm_speed=-100)
        print(f"Moved backward {distance_moved:.2f} mm")

        epuck.close()
    else:
        print("Failed to connect to e-puck.")
