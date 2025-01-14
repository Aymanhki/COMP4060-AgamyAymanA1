import math

# Constants (values will depend on your robot, adjust as needed)
WHEEL_DIAMETER_MM = 41  # Diameter of the wheel in mm (measured with calipers)
WHEEL_RADIUS_MM = WHEEL_DIAMETER_MM / 2
AXLE_LENGTH_MM = 53  # Distance between the wheels (measured with calipers)
STEPS_PER_REVOLUTION = 1000  # Motor step resolution per wheel rotation (check your robot specs)

# Helper function 1: Calculate step delta with wraparound
# Handles counter overflow in motor step readings
def steps_delta(last, current):
    MAX_STEP_COUNT = 2**15  # Assuming a 16-bit counter
    delta = current - last
    if delta > MAX_STEP_COUNT // 2:
        delta -= MAX_STEP_COUNT
    elif delta < -MAX_STEP_COUNT // 2:
        delta += MAX_STEP_COUNT
    return delta

# Helper function 2: Convert motor steps to radians
def steps_to_rad(steps):
    return (2 * math.pi * steps) / STEPS_PER_REVOLUTION

# Helper function 3: Convert radians to motor steps
def rad_to_steps(rad):
    return int((rad * STEPS_PER_REVOLUTION) / (2 * math.pi))

# Helper function 4: Convert radians of wheel rotation to ground distance (mm)
def rad_to_mm(rad):
    return rad * WHEEL_RADIUS_MM

# Helper function 5: Convert ground distance (mm) to wheel rotation (radians)
def mm_to_rad(mm):
    return mm / WHEEL_RADIUS_MM

# Helper function 6: Convert motor steps to ground distance (mm)
def steps_to_mm(steps):
    rad = steps_to_rad(steps)
    return rad_to_mm(rad)

# Helper function 7: Convert ground distance (mm) to motor steps
def mm_to_steps(mm):
    rad = mm_to_rad(mm)
    return rad_to_steps(rad)

# Helper function 8: Print robot pose (x_mm, y_mm, theta_rad)
def print_pose(pose):
    x_mm, y_mm, theta_rad = pose
    theta_deg = math.degrees(theta_rad)
    print(f"Pose: x={x_mm:.2f} mm, y={y_mm:.2f} mm, theta={theta_deg:.2f}°")

# Example usage
if __name__ == "__main__":
    # Example inputs
    last_steps = 200
    current_steps = 500

    print("Step delta:", steps_delta(last_steps, current_steps))
    print("Steps to radians:", steps_to_rad(100))
    print("Radians to steps:", rad_to_steps(math.pi / 2))
    print("Radians to mm:", rad_to_mm(math.pi / 2))
    print("mm to radians:", mm_to_rad(100))
    print("Steps to mm:", steps_to_mm(100))
    print("mm to steps:", mm_to_steps(100))

    # Print a sample pose
    print_pose((100, 200, math.pi / 4))
