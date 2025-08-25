#include "mav_trajectory_generation/example/example_planner.h"
#include "mav_trajectory_generation/polynomial_optimization_nonlinear.h"
#include <iostream>
#include <string>
#include <vector>

int main(int argc, char** argv) {
  if (argc < 10) {
    std::cerr << "Usage: " << argv[0] << " start_pos_x start_pos_y start_pos_z start_vel_x start_vel_y start_vel_z "
              << "goal_pos_x goal_pos_y goal_pos_z goal_vel_x goal_vel_y goal_vel_z output_file" << std::endl;
    return 1;
  }

  // Parse command line arguments
  std::vector<double> start_pos(3);
  std::vector<double> start_vel(3);
  std::vector<double> goal_pos(3);
  std::vector<double> goal_vel(3);

  for (int i = 0; i < 3; ++i) {
    start_pos[i] = std::stod(argv[i+1]);
    start_vel[i] = std::stod(argv[i+4]);
    goal_pos[i] = std::stod(argv[i+7]);
    goal_vel[i] = std::stod(argv[i+10]);
  }

  std::string output_file = argv[13];

  // Create planner and set parameters
  ExamplePlanner planner;
  planner.setMaxSpeed(2.0); // Set your desired max speed
  planner.setMaxAcceleration(5.0);

  // Set start position and velocity
  Eigen::Vector3d start_position(start_pos[0], start_pos[1], start_pos[2]);
  Eigen::Vector3d start_velocity(start_vel[0], start_vel[1], start_vel[2]);

  // Set goal position and velocity
  Eigen::Vector3d goal_position(goal_pos[0], goal_pos[1], goal_pos[2]);
  Eigen::Vector3d goal_velocity(goal_vel[0], goal_vel[1], goal_vel[2]);

  // Plan trajectory
  mav_trajectory_generation::Trajectory trajectory;
  bool success = planner.planTrajectory(goal_position, goal_velocity,
                                      start_position, start_velocity,
                                      planner.getMaxSpeed(), planner.getMaxAcceleration(),
                                      &trajectory);

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