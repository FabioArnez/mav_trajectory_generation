import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from trajectory_planner import ExamplePlanner4D, Trajectory, derivative_order

def plot_trajectory(trajectory):
    """Plot the 3D trajectory"""
    # Sample the trajectory at regular intervals
    t_values = np.linspace(0, trajectory.getMaxTime(), 100)
    positions = []

    for t in t_values:
        # Evaluate position at time t
        pos = trajectory.evaluate(t, derivative_order["POSITION"])
        positions.append(pos[:3])  # Only take x, y, z (ignore yaw)

    positions = np.array(positions)

    # Create figure
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')

    # Plot the trajectory
    ax.plot(positions[:, 0], positions[:, 1], positions[:, 2],
            label='Trajectory', linewidth=2)

    # Plot start and end points
    ax.scatter(positions[0, 0], positions[0, 1], positions[0, 2],
               color='green', s=100, label='Start')
    ax.scatter(positions[-1, 0], positions[-1, 1], positions[-1, 2],
               color='red', s=100, label='End')

    # Set labels and title
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_zlabel('Z (m)')
    ax.set_title('3D Trajectory Visualization')

    # Add legend
    ax.legend()

    # Set equal aspect ratio
    ax.set_box_aspect([1, 1, 1])

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
    current_velocity = np.array([1.0, 1.0, 0.0])
    planner.setCurrentVelocity(current_velocity)

    # Define goal position and velocity
    goal_pos = np.array([1.0, 2.0, 3.0, 0.0])  # x, y, z, yaw
    goal_vel = np.array([0.0, 0.0, 0.0, 0.0])  # vx, vy, vz, yaw_vel

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

    # Save trajectory to CSV
    output_file = "test_trajectory.csv"
    if planner.saveTrajectoryToCSV(trajectory, output_file):
        print(f"\nTrajectory saved to {output_file}")
    else:
        print("\nFailed to save trajectory to CSV")

    return True

if __name__ == "__main__":
    test_planner()