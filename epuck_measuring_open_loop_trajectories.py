import time
import numpy as np
import matplotlib.pyplot as plt
from epuck_open_loop_forward_kinematics import diff_drive_forward_kin
from epuck_simple_open_loop_controller import move_straight

# Task 5: Measure Open-loop Trajectories
def measure_trajectory(epuck, distance_mm, speed_mm_s, Hz, trials=10):
    """
    Measure the robot's trajectory error over multiple trials.

    Args:
        epuck: EPuck communication object.
        distance_mm: Target distance in mm.
        speed_mm_s: Target speed in mm/s.
        Hz: Control loop frequency.
        trials: Number of trials to run.

    Returns:
        List of error tuples [(err_x, err_y), ...] for each trial.
    """
    errors = []
    theoretical_pose = (0, distance_mm, 0)  # Ideal final pose

    for trial in range(trials):
        print(f"Trial {trial + 1}/{trials}:")

        # Reset initial pose
        initial_pose = (0, 0, 0)
        epuck.state.sens_left_motor_steps = 0
        epuck.state.sens_right_motor_steps = 0

        # Move the robot
        final_distance = move_straight(epuck, distance_mm, Hz)
        print(f"  Distance moved: {final_distance:.2f} mm")

        # Get the final pose (odometry)
        final_pose = diff_drive_forward_kin(initial_pose,
                                            epuck.state.sens_left_motor_steps,
                                            epuck.state.sens_right_motor_steps)

        # Calculate errors
        err_x = final_pose[0] - theoretical_pose[0]
        err_y = final_pose[1] - theoretical_pose[1]
        errors.append((err_x, err_y))

        print(f"  Error: err_x={err_x:.2f}, err_y={err_y:.2f}\n")

        time.sleep(2)  # Wait before the next trial

    return errors

# Plot errors
def plot_errors(errors, title):
    """
    Plot error scatter plot.

    Args:
        errors: List of error tuples [(err_x, err_y), ...].
        title: Title for the plot.
    """
    errors = np.array(errors)
    err_x = errors[:, 0]
    err_y = errors[:, 1]

    plt.figure(figsize=(8, 6))
    plt.scatter(err_x, err_y, c='blue', label='Errors', alpha=0.7)
    plt.axhline(0, color='red', linestyle='--', label='Ideal y-axis')
    plt.axvline(0, color='green', linestyle='--', label='Ideal x-axis')
    plt.xlabel("Error in x (mm)")
    plt.ylabel("Error in y (mm)")
    plt.title(title)
    plt.legend()
    plt.grid(True)
    plt.show()

# Main program
if __name__ == "__main__":
    from epuck_com import EPuckCom

    epuck = EPuckCom("COM16", debug=True)

    if epuck.connect():
        print("Connected to e-puck!")

        # Experiment parameters
        distance_mm = 1000
        speed_mm_s = 100
        frequencies = [1, 10, 30]  # Hz

        for Hz in frequencies:
            print(f"\nRunning trials at {Hz} Hz...")
            errors = measure_trajectory(epuck, distance_mm, speed_mm_s, Hz)
            plot_errors(errors, title=f"Trajectory Errors at {Hz} Hz")

        # Best frequency test with higher speed
        print("\nRunning high-speed trials at best frequency (30 Hz)...")
        high_speed_errors = measure_trajectory(epuck, distance_mm, 130, 30)
        plot_errors(high_speed_errors, title="High-speed Trajectory Errors at 30 Hz")

        epuck.close()
    else:
        print("Failed to connect to e-puck.")
