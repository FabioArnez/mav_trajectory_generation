#include "mav_trajectory_generation/example/example_planner.h"
#include "mav_trajectory_generation/polynomial_optimization_nonlinear.h"
#include <iostream>
#include <string>
#include <vector>

int main(int argc, char** argv) {
  if (argc < 19) {
    std::cerr << "Usage: " << argv[0] << " start_pos_x start_pos_y start_pos_z start_rot_x start_rot_y start_rot_z "
              << "start_vel_x start_vel_y start_vel_z start_ang_vel_x start_ang_vel_y start_ang_vel_z "
              << "goal_pos_x goal_pos_y goal_pos_z goal_rot_x goal_rot_y goal_rot_z "
              << "goal_vel_x goal_vel_y goal_vel_z goal_ang_vel_x goal_ang_vel_y goal_ang_vel_z output_file" << std::endl;
    return 1;
  }

  // Parse command line arguments
  std::vector<double> start_pos(3);
  std::vector<double> start_rot(3);
  std::vector<double> start_vel(3);
  std::vector<double> start_ang_vel(3);
  std::vector<double> goal_pos(3);
  std::vector<double> goal_rot(3);
  std::vector<double> goal_vel(3);
  std::vector<double> goal_ang_vel(3);

  for (int i = 0; i < 3; ++i) {
    start_pos[i] = std::stod(argv[i+1]);
    start_rot[i] = std::stod(argv[i+4]);
    start_vel[i] = std::stod(argv[i+7]);
    start_ang_vel[i] = std::stod(argv[i+10]);
    goal_pos[i] = std::stod(argv[i+13]);
    goal_rot[i] = std::stod(argv[i+16]);
    goal_vel[i] = std::stod(argv[i+19]);
    goal_ang_vel[i] = std::stod(argv[i+22]);
  }

  std::string output_file = argv[25];

  // Create planner and set parameters
  ExamplePlanner planner;
  planner.setMaxSpeed(2.0); // Set your desired max speed
  planner.setMaxAcceleration(2.0);
  planner.setMaxAngularSpeed(1.0);
  planner.setMaxAngularAcceleration(2.0);

  // Set start position, rotation, velocity, and angular velocity
  Eigen::Vector3d start_position(start_pos[0], start_pos[1], start_pos[2]);
  Eigen::Vector3d start_rotation(start_rot[0], start_rot[1], start_rot[2]);
  Eigen::Vector3d start_velocity(start_vel[0], start_vel[1], start_vel[2]);
  Eigen::Vector3d start_angular_velocity(start_ang_vel[0], start_ang_vel[1], start_ang_vel[2]);

  // Convert rotation vector to rotation matrix
  Eigen::Matrix3d rotation_mat;
  rotation_mat = Eigen::AngleAxisd(start_rotation.norm(), start_rotation.normalized());

  // Set current pose and velocity
  Eigen::Affine3d current_pose;
  current_pose.translation() = start_position;
  current_pose.linear() = rotation_mat;
  planner.setCurrentPose(current_pose);
  planner.setCurrentVelocity(start_velocity);
  planner.setCurrentAngularVelocity(start_angular_velocity);

  // Set goal position, rotation, velocity, and angular velocity
  Eigen::Vector3d goal_position(goal_pos[0], goal_pos[1], goal_pos[2]);
  Eigen::Vector3d goal_rotation(goal_rot[0], goal_rot[1], goal_rot[2]);
  Eigen::Vector3d goal_velocity(goal_vel[0], goal_vel[1], goal_vel[2]);
  Eigen::Vector3d goal_angular_velocity(goal_ang_vel[0], goal_ang_vel[1], goal_ang_vel[2]);

  // Create goal pose and twist vectors
  Eigen::VectorXd goal_pose(6), goal_twist(6);
  goal_pose << goal_position, goal_rotation;
  goal_twist << goal_velocity, goal_angular_velocity;

  // Plan trajectory
  mav_trajectory_generation::Trajectory trajectory;
  bool success = planner.planTrajectory(goal_pose, goal_twist, &trajectory);

  if (!success) {
    std::cerr << "Failed to plan trajectory" << std::endl;
    return 1;
  }

  // Save trajectory to CSV
  if (!planner.saveTrajectoryToCSV(trajectory, output_file)) {
    std::cerr << "Failed to save trajectory to CSV" << std::endl;
    return 1;
  }

  std::cout << "Trajectory successfully saved to " << output_file << std::endl;
  return 0;
}