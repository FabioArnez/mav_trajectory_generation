#include "mav_trajectory_generation/example/example_planner_4dof.h"
#include "mav_trajectory_generation/polynomial_optimization_nonlinear.h"
#include <iostream>
#include <string>
#include <vector>

int main(int argc, char** argv) {
  if (argc < 13) {
    std::cerr << "Usage: " << argv[0] << " start_pos_x start_pos_y start_pos_z start_yaw "
              << "start_vel_x start_vel_y start_vel_z start_yaw_vel "
              << "goal_pos_x goal_pos_y goal_pos_z goal_yaw "
              << "goal_vel_x goal_vel_y goal_vel_z goal_yaw_vel output_file" << std::endl;
    return 1;
  }

  // Parse command line arguments
  std::vector<double> start_pos(3);
  std::vector<double> start_vel(3);
  std::vector<double> goal_pos(3);
  std::vector<double> goal_vel(3);
  double start_yaw, start_yaw_vel, goal_yaw, goal_yaw_vel;

  for (int i = 0; i < 3; ++i) {
    start_pos[i] = std::stod(argv[i+1]);
    start_vel[i] = std::stod(argv[i+5]);
    goal_pos[i] = std::stod(argv[i+9]);
    goal_vel[i] = std::stod(argv[i+13]);
  }

  start_yaw = std::stod(argv[4]);
  start_yaw_vel = std::stod(argv[8]);
  goal_yaw = std::stod(argv[12]);
  goal_yaw_vel = std::stod(argv[16]);

  std::string output_file = argv[17];

  // Create planner and set parameters
  ExamplePlanner4D planner;
  planner.setMaxSpeed(2.0); // Set your desired max speed
  planner.setMaxAcceleration(2.0);

  // Set start position, yaw, velocity, and yaw velocity
  Eigen::Vector3d start_position(start_pos[0], start_pos[1], start_pos[2]);
  Eigen::Vector4d start_velocity(start_vel[0], start_vel[1], start_vel[2], start_yaw_vel);

  // Convert yaw to rotation matrix
  Eigen::Matrix3d rotation_mat;
  rotation_mat = Eigen::AngleAxisd(start_yaw, Eigen::Vector3d::UnitZ());

  // Set current pose and velocity
  Eigen::Affine3d current_pose;
  current_pose.translation() = start_position;
  current_pose.linear() = rotation_mat;
  planner.setCurrentPose(current_pose);
  planner.setCurrentVelocity(start_velocity);

  // Set goal position, yaw, velocity, and yaw velocity
  Eigen::Vector3d goal_position(goal_pos[0], goal_pos[1], goal_pos[2]);
  Eigen::Vector3d goal_velocity(goal_vel[0], goal_vel[1], goal_vel[2]);

  // Create goal pose and twist vectors
  Eigen::VectorXd goal_pose(4), goal_twist(4);
  goal_pose << goal_position, goal_yaw;
  goal_twist << goal_velocity, goal_yaw_vel;

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