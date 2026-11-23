import matplotlib.pyplot as plt
# Import Matplotlib to create and save the simulation graphs.
import numpy as np
# Import NumPy to handle arrays and numerical calculations.

DT = 0.02
# Set each simulation step to 20 milliseconds.
WHEEL_BASE = 0.18
# Set the distance between the left and right wheels to 0.18 metres.
SENSOR_FORWARD = 0.12
# Place the sensors 0.12 metres in front of the robot centre.
SENSOR_OFFSETS = np.linspace(-0.08, 0.08, 5)
# Define five sensor positions evenly spaced from left to right across the robot.
SENSOR_SIGMA = 0.045
# Set how quickly sensor readings decrease when the sensor moves away from the line.
BASE_SPEED = 0.38
# Set the normal forward speed to 0.38 metres per second.
MAX_WHEEL_SPEED = 0.55
# Limit each wheel to a maximum speed of 0.55 metres per second.

CONTROLLERS = {
# Store the controller configurations to compare.
    "P-only":    {"kp": 3.5, "ki": 0.0,   "kd": 0.0},
# Configure P-only control with no integral or derivative action.
    "Tuned PID": {"kp": 2.8, "ki": 0.003, "kd": 0.01},
# Configure tuned PID control with proportional, integral, and derivative gains.
}


def build_track() -> np.ndarray:
# Create and return an array containing every point in the track.
    straight_x = np.linspace(0.0, 1.0, 120, endpoint=False)
# Generate 120 evenly spaced x-coordinates for the starting straight section.
    straight = np.column_stack((straight_x, np.zeros_like(straight_x)))
# Give the straight section y-coordinate zero for every point.

    s_x = np.linspace(1.0, 3.8, 420, endpoint=False)
# Generate 420 x-coordinates for the S-shaped section.
    s_y = 0.38 * np.sin(2.0 * np.pi * (s_x - 1.0) / 2.8)
# Calculate smooth sine-wave y-coordinates for the S-shaped section.
    s_curve = np.column_stack((s_x, s_y))
# Combine the S-section x-coordinates and y-coordinates into track points.

    approach_x = np.linspace(3.8, 4.6, 100, endpoint=False)
# Generate x-coordinates for the straight approach before the turn.
    approach = np.column_stack((approach_x, np.full_like(approach_x, s_y[-1])))
# Keep the approach section at the final y-coordinate of the S-curve.

    turn_radius = 0.8
# Set the 90-degree turn radius to 0.8 metres.
    turn_center = np.array([4.6, s_y[-1] + turn_radius])
# Calculate the centre of the circular turn.
    turn_angle = np.linspace(-np.pi / 2.0, 0.0, 100, endpoint=False)
# Generate 100 angles that trace the turn from bottom to right.
    turn = turn_center + turn_radius * np.column_stack((np.cos(turn_angle), np.sin(turn_angle)))
# Convert each turn angle and the radius into x/y track coordinates.

    vertical_y = np.linspace(turn_center[1] + turn_radius, turn_center[1] + 1.5, 180)
# Generate y-coordinates for the final vertical section.
    vertical = np.column_stack((np.full_like(vertical_y, turn_center[0] + turn_radius), vertical_y))
# Keep x fixed for the final vertical section while varying y.
    return np.vstack((straight, s_curve, approach, turn, vertical))
# Join all track sections into one continuous array.


def nearest_distances(points: np.ndarray, track: np.ndarray) -> np.ndarray:
# Find the nearest track distance for each supplied point.
    diff = points[:, np.newaxis, :] - track[np.newaxis, :, :]
# Subtract every track point from every supplied point.
    return np.sqrt(np.min(np.sum(diff ** 2, axis=2), axis=1))
# Square and sum the differences, then return the nearest distance for each point.


def simulate(gains: dict[str, float], track: np.ndarray) -> dict[str, np.ndarray]:
# Run the complete simulation using one controller configuration.
    state = np.array([0.0, 0.0, 0.0])  # [x, y, heading]
# Initialize the robot's x-position, y-position, and heading.
    integral = previous_error = filtered_derivative = 0.0
# Initialize the integral, previous error, and filtered derivative to zero.
    derivative_alpha = 0.25
# Set the derivative smoothing factor to 25 percent.

    times, positions, errors, deviations, wheel_speeds = [], [], [], [], []
# Create empty lists for the data that will be recorded during the simulation.

    for step in range(int(24.0 / DT)):
# Repeat the simulation loop for a maximum of 24 seconds.
        x, y, heading = state
# Read the robot's current position and heading from the state array.
        forward = np.array([np.cos(heading), np.sin(heading)])
# Calculate the unit vector pointing straight ahead of the robot.
        left = np.array([-np.sin(heading), np.cos(heading)])
# Calculate the unit vector pointing left from the robot.
        sensor_points = (state[:2] + SENSOR_FORWARD * forward) + SENSOR_OFFSETS[:, np.newaxis] * left
# Place all five sensors across the robot and 0.12 metres in front of it.

        all_dists = nearest_distances(np.vstack((state[:2], sensor_points)), track)
# Calculate both the robot's deviation and every sensor's distance from the track.
        deviation, sensor_dists = float(all_dists[0]), all_dists[1:]
# Separate the robot deviation from the five individual sensor distances.

        readings = np.exp(-0.5 * (sensor_dists / SENSOR_SIGMA) ** 2)
# Convert sensor distances into readings that become stronger near the track line.
        total_signal = float(np.sum(readings))
# Sum all five readings to obtain the total line signal.
        error = float(np.dot(readings, SENSOR_OFFSETS) / total_signal / 0.08) if total_signal > 1e-8 else previous_error
# Calculate normalized line position, or reuse the previous error if no sensor sees the line.

        derivative = (error - previous_error) / DT
# Calculate how quickly the normalized error is changing.
        filtered_derivative += derivative_alpha * (derivative - filtered_derivative)
# Smooth the derivative to reduce noise from sudden error changes.
        integral = float(np.clip(integral + error * DT, -2.0, 2.0))
# Accumulate error over time and prevent the integral from growing beyond safe limits.

        angular_speed = np.clip(
# Start building the limited steering speed.
            gains["kp"] * error + gains["ki"] * integral + gains["kd"] * filtered_derivative,
# Combine proportional, integral, and derivative PID terms.
            -3.2, 3.2
# Restrict the steering speed to between -3.2 and 3.2 radians per second.
        )
# Finish the angular-speed calculation.

        left_speed = float(np.clip(BASE_SPEED - angular_speed * WHEEL_BASE / 2.0, -MAX_WHEEL_SPEED, MAX_WHEEL_SPEED))
# Calculate and limit the left-wheel speed.
        right_speed = float(np.clip(BASE_SPEED + angular_speed * WHEEL_BASE / 2.0, -MAX_WHEEL_SPEED, MAX_WHEEL_SPEED))
# Calculate and limit the right-wheel speed.

        times.append(step * DT)
# Record the current simulation time.
        positions.append(state[:2].copy())
# Record a copy of the robot's current x/y position.
        errors.append(error)
# Record the normalized sensor error.
        deviations.append(deviation)
# Record the robot's distance from the track centerline.
        wheel_speeds.append((left_speed, right_speed))
# Record the left and right wheel speeds.

        linear_speed = (left_speed + right_speed) / 2.0
# Calculate the average speed of the two wheels.
        actual_omega = (right_speed - left_speed) / WHEEL_BASE
# Calculate the robot's actual turning speed from the wheel-speed difference.
        state[0] += linear_speed * np.cos(heading) * DT
# Update the robot's x-position.
        state[1] += linear_speed * np.sin(heading) * DT
# Update the robot's y-position.
        state[2] += actual_omega * DT
# Update the robot's heading.
        previous_error = error
# Save the current error for use in the next derivative calculation.

        if np.linalg.norm(state[:2] - track[-1]) < 0.08:
# Check whether the robot is within 8 centimetres of the finish point.
            break
# Exit the simulation loop after reaching the finish.

    return {
# Return the recorded simulation data.
        "time": np.array(times),
# Convert the recorded times into a NumPy array.
        "position": np.array(positions),
# Convert the recorded positions into a NumPy array.
        "sensor_error": np.array(errors),
# Convert the recorded sensor errors into a NumPy array.
        "cross_track_error": np.array(deviations),
# Convert the recorded track distances into a NumPy array.
        "wheel_speeds": np.array(wheel_speeds),
# Convert the recorded wheel speeds into a NumPy array.
    }


def make_plot(track: np.ndarray, results: dict[str, dict[str, np.ndarray]]) -> None:
# Create the comparison figure and save it as an image.
    colors = {"P-only": "#e76f51", "Tuned PID": "#2878b5"}
# Assign a distinct color to each controller.
    fig, axes = plt.subplots(2, 2, figsize=(13, 9), constrained_layout=True)
# Create a 13-by-9-inch figure with four plots in a 2-by-2 layout.
    path_ax, error_ax, sensor_ax, speed_ax = axes.flat
# Assign each plot area to a descriptive variable.

    path_ax.plot(track[:, 0], track[:, 1], color="black", linewidth=2.5, label="Track centerline")
# Draw the track centerline in black.
    for name, res in results.items():
# Repeat the path plotting for each controller.
        path_ax.plot(res["position"][:, 0], res["position"][:, 1], color=colors[name], linewidth=1.7,
# Draw the robot's travelled path with its controller color.
                     label=f'{name} (mean |err| {np.mean(res["cross_track_error"]):.3f} m)')
# Add the controller name and mean track error to the path legend.
    path_ax.scatter(*track[0], color="green", marker="o", s=45, zorder=4, label="Start")
# Mark the starting point with a green circle.
    path_ax.scatter(*track[-1], color="black", marker="x", s=55, zorder=4, label="Finish")
# Mark the finish point with a black X.
    path_ax.set(title="Robot paths on S-curve and 90° turn", xlabel="x (m)", ylabel="y (m)")
# Set the robot-path title and axis labels.
    path_ax.axis("equal")
# Use the same scale for horizontal and vertical distances.
    path_ax.grid(alpha=0.25)
# Add faint grid lines to the path plot.
    path_ax.legend(fontsize=8)
# Show the path plot legend using small text.

    for name, res in results.items():
# Repeat the three data plots for each controller.
        error_ax.plot(res["time"], res["cross_track_error"], color=colors[name], label=name)
# Plot cross-track error against time.
        sensor_ax.plot(res["time"], res["sensor_error"], color=colors[name], label=name)
# Plot normalized sensor error against time.
        speed_ax.plot(res["time"], res["wheel_speeds"][:, 0], color=colors[name], linestyle="-", label=f"{name} left")
# Plot left-wheel speed as a solid line.
        speed_ax.plot(res["time"], res["wheel_speeds"][:, 1], color=colors[name], linestyle="--", label=f"{name} right")
# Plot right-wheel speed as a dashed line.

    error_ax.set(title="Absolute cross-track error", xlabel="Time (s)", ylabel="Distance from track (m)")
# Set the title and axis labels for the cross-track error plot.
    sensor_ax.axhline(0.0, color="black", linewidth=0.8, alpha=0.5)
# Draw a horizontal zero-error reference line on the sensor plot.
    sensor_ax.set(title="Normalized 5-sensor error", xlabel="Time (s)", ylabel="Weighted position")
# Set the title and axis labels for the sensor-error plot.
    speed_ax.set(title="Differential-drive wheel speeds", xlabel="Time (s)", ylabel="Speed (m/s)")
# Set the title and axis labels for the wheel-speed plot.

    for ax in (error_ax, sensor_ax, speed_ax):
# Apply common settings to the three time-based plots.
        ax.grid(alpha=0.3)
# Add faint grid lines to the plot.
        ax.legend()
# Add a legend to the plot.

    fig.suptitle("Differential-drive line follower: P-only vs tuned PID", fontsize=15)
# Set the main title for the entire figure.
    fig.savefig("line_follower_pid_comparison.png", dpi=160)
# Save the figure as a 160-DPI PNG image.
    plt.close(fig)
# Close the Matplotlib figure after saving it.


if __name__ == "__main__":
# Run the following block only when this file is executed directly.
    track = build_track()
# Build the complete track.
    results = {name: simulate(gains, track) for name, gains in CONTROLLERS.items()}
# Run the simulation once for every controller.
    make_plot(track, results)
# Create and save the comparison graphs.
    for name, res in results.items():
# Repeat the summary for every controller result.
        print(f'{name}: mean={np.mean(res["cross_track_error"]):.3f} m, max={np.max(res["cross_track_error"]):.3f} m')
# Print the average and maximum track deviation for the controller.
