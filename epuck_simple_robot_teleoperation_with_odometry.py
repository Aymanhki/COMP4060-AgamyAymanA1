# TASK 3: Simple robot teleoperation with odometry

import time
from threading import Thread
from pynput import keyboard
from epuck_open_loop_forward_kinematics import diff_drive_forward_kin
from epuck_helper_functions import print_pose
from epuck_com import EPuckCom
from epuck_ip import EPuckIP

# Global variables for keyboard input
key_states = {"w": False, "s": False, "a": False, "d": False}

# Keyboard listener thread
def on_press(key):
    try:
        if key.char in key_states:
            key_states[key.char] = True
    except AttributeError:
        pass

def on_release(key):
    try:
        if key.char in key_states:
            key_states[key.char] = False
    except AttributeError:
        pass

# Robot teleoperation with odometry
def teleoperate_robot(epuck, initial_pose=(0, 0, 0), Hz=10):
    """
    Teleoperate the robot using keyboard input and calculate its pose with odometry.

    Args:
        epuck: EPuck communication object.
        initial_pose: Tuple (x, y, theta), initial robot pose.
        Hz: Control loop frequency.
    """
    current_pose = initial_pose
    left_steps_last = epuck.state.sens_left_motor_steps
    right_steps_last = epuck.state.sens_right_motor_steps

    loop_interval = 1 / Hz
    time_elapsed = 0

    try:
        while True:
            # Map key states to motor speeds
            if key_states["w"] and key_states["a"]:  # Move diagonally forward-left
                l_speed = 300  # Slower forward for left motor
                r_speed = 500  # Faster forward for right motor
            elif key_states["w"] and key_states["d"]:  # Move diagonally forward-right
                l_speed = 500  # Faster forward for left motor
                r_speed = 300  # Slower forward for right motor
            elif key_states["s"] and key_states["a"]:  # Move diagonally backward-left
                l_speed = -300  # Slower backward for left motor
                r_speed = -500  # Faster backward for right motor
            elif key_states["s"] and key_states["d"]:  # Move diagonally backward-right
                l_speed = -500  # Faster backward for left motor
                r_speed = -300  # Slower backward for right motor
            elif key_states["w"]:  # Forward
                l_speed = r_speed = 500
            elif key_states["s"]:  # Backward
                l_speed = r_speed = -500
            elif key_states["a"]:  # Turn left
                l_speed = -300
                r_speed = 300
            elif key_states["d"]:  # Turn right
                l_speed = 300
                r_speed = -300
            else:  # Stop
                l_speed = r_speed = 0

            # Send motor commands
            epuck.state.act_left_motor_speed = l_speed
            epuck.state.act_right_motor_speed = r_speed
            epuck.send_command()

            # Update sensor data
            epuck.data_update()
            left_steps_current = epuck.state.sens_left_motor_steps
            right_steps_current = epuck.state.sens_right_motor_steps

            # Calculate step deltas
            left_delta = left_steps_current - left_steps_last
            right_delta = right_steps_current - right_steps_last
            left_steps_last = left_steps_current
            right_steps_last = right_steps_current

            # Update robot pose
            current_pose = diff_drive_forward_kin(current_pose, left_delta, right_delta)

            # Print pose every second
            time_elapsed += loop_interval
            if time_elapsed >= 1.0:
                print_pose(current_pose)
                time_elapsed = 0

            time.sleep(loop_interval)
    except KeyboardInterrupt:
        print("Exiting teleoperation.")
        epuck.stop_all()

# Main program
if __name__ == "__main__":
    # epuck = EPuckCom("COM16", debug=True)
    epuck = EPuckIP("172.20.10.4", debug=True)
    epuck.enable_sensors = True

    if epuck.connect():
        print("Connected to e-puck! Use WASD keys to control the robot.")

        # Start keyboard listener
        listener = keyboard.Listener(on_press=on_press, on_release=on_release)
        listener.start()

        # Start teleoperation
        teleoperate_robot(epuck)

        listener.stop()
        epuck.close()
    else:
        print("Failed to connect to e-puck.")
