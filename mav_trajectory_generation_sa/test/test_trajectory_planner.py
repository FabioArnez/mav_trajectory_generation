import numpy as np
import matplotlib.pyplot as plt
from math import pi
from mpl_toolkits.mplot3d import Axes3D
from trajectory_planner import ExamplePlanner4D, Trajectory, derivative_order


def plot_trajectory(trajectory):
    """Plot the 3D trajectory with orientation arrows"""
    # Sample the trajectory at regular intervals
    t_values = np.linspace(0, trajectory.getMaxTime(), 20)  # Fewer points for arrows
    positions = []
    orientations = []

    for t in t_values:
        # Evaluate position at time t
        pos = trajectory.evaluate(t, derivative_order["POSITION"])
        positions.append(pos[:3])  # Only take x, y, z (ignore yaw)

        # Evaluate orientation (yaw) at time t
        # Note: This assumes the last component is yaw
        yaw = pos[3] if len(pos) > 3 else 0.0
        orientations.append(yaw)

    positions = np.array(positions)
    orientations = np.array(orientations)

    # Create figure
    fig = plt.figure(figsize=(12, 10))
    ax = fig.add_subplot(111, projection='3d')

    # Plot the trajectory
    ax.plot(positions[:, 0], positions[:, 1], positions[:, 2],
            label='Trajectory', linewidth=2, color='blue')

    # Plot start and end points
    ax.scatter(positions[0, 0], positions[0, 1], positions[0, 2],
               color='green', s=100, label='Start')
    ax.scatter(positions[-1, 0], positions[-1, 1], positions[-1, 2],
               color='red', s=100, label='End')

    # Add orientation arrows
    arrow_length = 0.2  # Length of the orientation arrows
    for i, (pos, yaw) in enumerate(zip(positions, orientations)):
        # Calculate arrow direction in 3D space
        # Assuming yaw is rotation around Z-axis
        dx = arrow_length * np.cos(yaw)
        dy = arrow_length * np.sin(yaw)
        dz = 0  # No change in Z for yaw

        # Plot the arrow
        ax.quiver(pos[0], pos[1], pos[2],
                 dx, dy, dz,
                 color='purple', length=arrow_length,
                 arrow_length_ratio=0.2, label='Orientation' if i == 0 else "")

    # Set labels and title
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_zlabel('Z (m)')
    ax.set_title('3D Trajectory with Orientation Visualization')

    # Set equal aspect ratio
    ax.set_box_aspect([1, 1, 1])

    # Calculate limits for x and y axes to ensure same scale
    x_min, x_max = positions[:, 0].min(), positions[:, 0].max()
    y_min, y_max = positions[:, 1].min(), positions[:, 1].max()

    # Calculate the range for x and y
    x_range = x_max - x_min
    y_range = y_max - y_min

    # Determine the maximum range to use for both axes
    max_range = max(x_range, y_range)

    # Calculate the center of the data
    x_center = (x_max + x_min) / 2
    y_center = (y_max + y_min) / 2

    # Set the limits to be the same for both axes
    ax.set_xlim([x_center - max_range/2, x_center + max_range/2])
    ax.set_ylim([y_center - max_range/2, y_center + max_range/2])

    # Add legend
    handles, labels = ax.get_legend_handles_labels()
    # Remove duplicate labels for arrows
    unique_labels = dict(zip(labels, handles))
    ax.legend(unique_labels.values(), unique_labels.keys())

    # Adjust view angle for better visualization
    ax.view_init(elev=20, azim=225)

    plt.tight_layout()
    plt.show()

def plot_trajectory_quadrotor(trajectory):
    """Plot the 3D trajectory with orientation arrows and quadrotor model"""
    # Sample the trajectory at regular intervals
    t_values = np.linspace(0, trajectory.getMaxTime(), 20)  # Fewer points for arrows
    positions = []
    orientations = []

    for t in t_values:
        # Evaluate position at time t
        pos = trajectory.evaluate(t, derivative_order["POSITION"])
        positions.append(pos[:3])  # Only take x, y, z (ignore yaw)

        # Evaluate orientation (yaw) at time t
        # Note: This assumes the last component is yaw
        yaw = pos[3] if len(pos) > 3 else 0.0
        orientations.append(yaw)

    positions = np.array(positions)
    orientations = np.array(orientations)

    # Create figure
    fig = plt.figure(figsize=(12, 10))
    ax = fig.add_subplot(111, projection='3d')

    # Plot the trajectory
    ax.plot(positions[:, 0], positions[:, 1], positions[:, 2],
            label='Trajectory', linewidth=2, color='blue')

    # Plot start and end points
    ax.scatter(positions[0, 0], positions[0, 1], positions[0, 2],
               color='green', s=100, label='Start')
    ax.scatter(positions[-1, 0], positions[-1, 1], positions[-1, 2],
               color='red', s=100, label='End')

    # Add quadrotor model at start position
    def plot_quadrotor(x, y, z, yaw, size=0.3):
        """Plot a simple quadrotor model at the given position and orientation"""
        # Body frame (square)
        body_vertices = np.array([
            [size, size, 0],
            [-size, size, 0],
            [-size, -size, 0],
            [size, -size, 0],
            [size, size, 0]  # Close the loop
        ])

        # Rotate the body frame according to yaw
        rotation_matrix = np.array([
            [np.cos(yaw), -np.sin(yaw), 0],
            [np.sin(yaw), np.cos(yaw), 0],
            [0, 0, 1]
        ])
        rotated_vertices = np.dot(body_vertices, rotation_matrix.T)

        # Translate to the desired position
        rotated_vertices[:, 0] += x
        rotated_vertices[:, 1] += y
        rotated_vertices[:, 2] += z

        # Plot the body
        ax.plot(rotated_vertices[:, 0], rotated_vertices[:, 1], rotated_vertices[:, 2],
                color='black', linewidth=2)

        # Plot the arms (simplified as lines)
        arm_length = size * 1.5
        arm_vertices = np.array([
            [arm_length, 0, 0],
            [-arm_length, 0, 0],
            [0, arm_length, 0],
            [0, -arm_length, 0]
        ])
        rotated_arms = np.dot(arm_vertices, rotation_matrix.T)
        rotated_arms[:, 0] += x
        rotated_arms[:, 1] += y
        rotated_arms[:, 2] += z

        # Plot the arms
        ax.plot([x, rotated_arms[0, 0]], [y, rotated_arms[0, 1]], [z, rotated_arms[0, 2]],
                color='black', linewidth=1)
        ax.plot([x, rotated_arms[1, 0]], [y, rotated_arms[1, 1]], [z, rotated_arms[1, 2]],
                color='black', linewidth=1)
        ax.plot([x, rotated_arms[2, 0]], [y, rotated_arms[2, 1]], [z, rotated_arms[2, 2]],
                color='black', linewidth=1)
        ax.plot([x, rotated_arms[3, 0]], [y, rotated_arms[3, 1]], [z, rotated_arms[3, 2]],
                color='black', linewidth=1)

        # Plot the propellers (simplified as circles)
        propeller_positions = np.array([
            [arm_length, 0, 0],
            [-arm_length, 0, 0],
            [0, arm_length, 0],
            [0, -arm_length, 0]
        ])
        rotated_propellers = np.dot(propeller_positions, rotation_matrix.T)
        rotated_propellers[:, 0] += x
        rotated_propellers[:, 1] += y
        rotated_propellers[:, 2] += z

        for prop in rotated_propellers:
            # Create a circle for each propeller
            theta = np.linspace(0, 2*np.pi, 20)
            circle_x = prop[0] + size*0.3 * np.cos(theta)
            circle_y = prop[1] + size*0.3 * np.sin(theta)
            circle_z = prop[2] * np.ones_like(theta)
            ax.plot(circle_x, circle_y, circle_z, color='gray', linewidth=1)

    # Plot quadrotor at start position
    start_yaw = orientations[0] if len(orientations) > 0 else 0.0
    plot_quadrotor(positions[0, 0], positions[0, 1], positions[0, 2], start_yaw)

    # Add orientation arrows
    arrow_length = 0.2  # Length of the orientation arrows
    for i, (pos, yaw) in enumerate(zip(positions, orientations)):
        # Calculate arrow direction in 3D space
        # Assuming yaw is rotation around Z-axis
        dx = arrow_length * np.cos(yaw)
        dy = arrow_length * np.sin(yaw)
        dz = 0  # No change in Z for yaw

        # Plot the arrow
        ax.quiver(pos[0], pos[1], pos[2],
                 dx, dy, dz,
                 color='purple', length=arrow_length,
                 arrow_length_ratio=0.2, label='Orientation' if i == 0 else "")

    # Set labels and title
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_zlabel('Z (m)')
    ax.set_title('3D Trajectory with Orientation Visualization')

    # Set equal aspect ratio
    ax.set_box_aspect([1, 1, 1])

    # Calculate limits for x and y axes to ensure same scale
    x_min, x_max = positions[:, 0].min(), positions[:, 0].max()
    y_min, y_max = positions[:, 1].min(), positions[:, 1].max()

    # Calculate the range for x and y
    x_range = x_max - x_min
    y_range = y_max - y_min

    # Determine the maximum range to use for both axes
    max_range = max(x_range, y_range)

    # Calculate the center of the data
    x_center = (x_max + x_min) / 2
    y_center = (y_max + y_min) / 2

    # Set the limits to be the same for both axes
    ax.set_xlim([x_center - max_range/2, x_center + max_range/2])
    ax.set_ylim([y_center - max_range/2, y_center + max_range/2])

    # Add legend
    handles, labels = ax.get_legend_handles_labels()
    # Remove duplicate labels for arrows
    unique_labels = dict(zip(labels, handles))
    ax.legend(unique_labels.values(), unique_labels.keys())

    # Adjust view angle for better visualization
    ax.view_init(elev=20, azim=225)

    plt.tight_layout()
    plt.show()


def test_planner():
    # Create planner instance
    planner = ExamplePlanner4D()

    # Set parameters
    planner.setMaxSpeed(2.0)
    planner.setMaxAcceleration(2.0)

    # Set current pose (identity pose)
    current_pose = np.eye(4)
    planner.setCurrentPose(current_pose)

    # Set current velocity
    current_velocity = np.array([1.0, 0.0, 0.0])
    # current_velocity = np.array([1.0, 0.0, 0.0, 0.0])
    planner.setCurrentVelocity(current_velocity)

    # Define goal position and velocity
    goal_pos = np.array([3.0, 1.0, 0.0, pi/4])  # x, y, z, yaw
    goal_vel = np.array([0.0, 0.0, 0.0, 0])  # vx, vy, vz, yaw_vel

    # Plan trajectory
    trajectory = Trajectory()
    success = planner.planTrajectory(goal_pos, goal_vel, trajectory)

    if not success:
        print("Failed to plan trajectory")
        return False

    print("Trajectory planned successfully!")
    print(f"Trajectory duration: {trajectory.getMaxTime()} seconds")

    # Plot the trajectory
    plot_trajectory(trajectory)
    # plot_trajectory_quadrotor(trajectory)

    # Save trajectory to CSV
    output_file = "test_trajectory.csv"
    if planner.saveTrajectoryToCSV(trajectory, output_file):
        print(f"\nTrajectory saved to {output_file}")
    else:
        print("\nFailed to save trajectory to CSV")

    return True

if __name__ == "__main__":
    test_planner()