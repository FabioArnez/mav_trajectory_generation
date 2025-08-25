#include "mav_trajectory_generation/example/example_planner.h"
#include "mav_trajectory_generation/polynomial_optimization_nonlinear.h"
#include <fstream>
#include <iomanip>

ExamplePlanner::ExamplePlanner()
    : max_v_(2.0),
      max_a_(2.0),
      max_ang_v_(2.0),
      max_ang_a_(2.0),
      current_velocity_(Eigen::Vector3d::Zero()),
      current_angular_velocity_(Eigen::Vector3d::Zero()),
      current_pose_(Eigen::Affine3d::Identity()) {
}

// Method to set maximum speed.
void ExamplePlanner::setMaxSpeed(const double max_v) {
  max_v_ = max_v;
}

// Method to get maximum speed.
double ExamplePlanner::getMaxSpeed() const {
  return max_v_;
}

// Method to set maximum acceleration.
void ExamplePlanner::setMaxAcceleration(const double max_a) {
  max_a_ = max_a;
}

// Method to get maximum acceleration.
double ExamplePlanner::getMaxAcceleration() const {
  return max_a_;
}

// Method to set maximum angular speed.
void ExamplePlanner::setMaxAngularSpeed(const double max_ang_v) {
  max_ang_v_ = max_ang_v;
}

// Method to get maximum angular speed.
double ExamplePlanner::getMaxAngularSpeed() const {
  return max_ang_v_;
}

// Method to set maximum angular acceleration.
void ExamplePlanner::setMaxAngularAcceleration(const double max_ang_a) {
  max_ang_a_ = max_ang_a;
}

// Method to get maximum angular acceleration.
double ExamplePlanner::getMaxAngularAcceleration() const {
  return max_ang_a_;
}

// Method to set current pose.
void ExamplePlanner::setCurrentPose(const Eigen::Affine3d& pose) {
  current_pose_ = pose;
}

// Method to set current velocity.
void ExamplePlanner::setCurrentVelocity(const Eigen::Vector3d& velocity) {
  current_velocity_ = velocity;
}

// Method to set current angular velocity.
void ExamplePlanner::setCurrentAngularVelocity(const Eigen::Vector3d& angular_velocity) {
  current_angular_velocity_ = angular_velocity;
}

// Plans a trajectory from the current position to the a goal position and velocity
bool ExamplePlanner::planTrajectory(const Eigen::VectorXd& goal_pos,
                                    const Eigen::VectorXd& goal_vel,
                                    mav_trajectory_generation::Trajectory* trajectory) {
  assert(trajectory);
  trajectory->clear();

  // 3 Dimensional trajectory => 3D position
  // 4 Dimensional trajectory => 3D position + yaw
  // 6 Dimensional trajectory => through SE(3) space, position and orientation
  const int dimension = goal_pos.size();
  bool success = false;

  if (dimension == 6)
  {
    mav_trajectory_generation::Trajectory trajectory_trans, trajectory_rot;

    // Translation trajectory.
    Eigen::Vector3d goal_position = goal_pos.head(3);
    Eigen::Vector3d goal_lin_vel = goal_vel.head(3);
    success = planTrajectory(
        goal_position, goal_lin_vel, current_pose_.translation(),
        current_velocity_, max_v_, max_a_, &trajectory_trans);

    // Rotation trajectory.
    Eigen::Vector3d goal_rotation = goal_pos.tail(3);
    Eigen::Vector3d goal_ang_vel = goal_vel.tail(3);
    Eigen::Vector3d current_rot_vec;
    // Convert rotation matrix to rotation vector
    Eigen::AngleAxisd aa(current_pose_.rotation());
    current_rot_vec = aa.axis() * aa.angle();

    success &= planTrajectory(
        goal_rotation, goal_ang_vel, current_rot_vec, current_angular_velocity_,
        max_ang_v_, max_ang_a_, &trajectory_rot);

    // Combine trajectories.
    success &= trajectory_trans.getTrajectoryWithAppendedDimension(
            trajectory_rot, &(*trajectory));
    return success;
  }
  else if (dimension == 3)
  {
    success = planTrajectory(
        goal_pos, goal_vel, current_pose_.translation(), current_velocity_,
        max_v_, max_a_, &(*trajectory));
    return success;
  }
  else if (dimension == 4)
  {
    Eigen::Vector4d start_pos_4d, start_vel_4d;
    start_pos_4d << current_pose_.translation(),
        Eigen::Quaterniond(current_pose_.rotation()).toRotationMatrix().eulerAngles(2, 1, 0)[0];
    start_vel_4d << current_velocity_, 0.0;
    success = planTrajectory(
        goal_pos, goal_vel, start_pos_4d, start_vel_4d, max_v_, max_a_,
        &(*trajectory));
    return success;
  }
  else
  {
    std::cerr << "Dimension must be 3, 4 or 6 to be valid." << std::endl;
    return false;
  }
}

// Plans a trajectory from a start position and velocity to a goal position and velocity
bool ExamplePlanner::planTrajectory(const Eigen::VectorXd& goal_pos,
                                    const Eigen::VectorXd& goal_vel,
                                    const Eigen::VectorXd& start_pos,
                                    const Eigen::VectorXd& start_vel,
                                    double v_max, double a_max,
                                    mav_trajectory_generation::Trajectory* trajectory) {
  assert(trajectory);
  const int dimension = goal_pos.size();
  // Array for all waypoints and their constraints
  mav_trajectory_generation::Vertex::Vector vertices;

  // Optimze up to 4th order derivative (SNAP)
  const int derivative_to_optimize =
      mav_trajectory_generation::derivative_order::SNAP;

  // we have 2 vertices:
  // start = desired start vector
  // end = desired end vector
  mav_trajectory_generation::Vertex start(dimension), end(dimension);

  /******* Configure start point *******/
  start.makeStartOrEnd(start_pos, derivative_to_optimize);
  start.addConstraint(mav_trajectory_generation::derivative_order::VELOCITY,
                      start_vel);
  vertices.push_back(start);

  /******* Configure end point *******/
  // set end point constraints to desired position and set all derivatives to zero
  end.makeStartOrEnd(goal_pos, derivative_to_optimize);
  end.addConstraint(mav_trajectory_generation::derivative_order::VELOCITY,
                    goal_vel);
  vertices.push_back(end);

  // setimate initial segment times
  std::vector<double> segment_times;
  segment_times = estimateSegmentTimes(vertices, v_max, a_max);

  // Set up polynomial solver with default params
  mav_trajectory_generation::NonlinearOptimizationParameters parameters;

  // set up optimization problem
  const int N = 10;
  mav_trajectory_generation::PolynomialOptimizationNonLinear<N> opt(dimension, parameters);
  opt.setupFromVertices(vertices, segment_times, derivative_to_optimize);

  // constrain velocity and acceleration
  opt.addMaximumMagnitudeConstraint(mav_trajectory_generation::derivative_order::VELOCITY, v_max);
  opt.addMaximumMagnitudeConstraint(mav_trajectory_generation::derivative_order::ACCELERATION, a_max);

  // solve trajectory
  opt.optimize();

  // get trajectory as polynomial parameters
  opt.getTrajectory(&(*trajectory));
  trajectory->scaleSegmentTimesToMeetConstraints(v_max, a_max);

  return true;
}

bool ExamplePlanner::saveTrajectoryToCSV(const mav_trajectory_generation::Trajectory& trajectory,
                                        const std::string& filename) {
  std::ofstream file(filename);
  if (!file.is_open()) {
    std::cerr << "Failed to open file: " << filename << std::endl;
    return false;
  }

  // Write header
  file << "time,";
  // Use trajectory.D() instead of trajectory.dimension()
  for (int i = 0; i < trajectory.D(); ++i) {
    file << "x" << i << ",";
  }
  file << std::endl;

  // Write trajectory data
  const double dt = 0.01; // Time step for sampling
  double t = 0.0;
  while (t <= trajectory.getMaxTime() + 1e-5) {
    Eigen::VectorXd position;
    // Use trajectory.evaluate(t) instead of trajectory.evaluate(t, derivative_order::POSITION, &position)
    position = trajectory.evaluate(t);

    file << std::fixed << std::setprecision(4) << t << ",";
    for (int i = 0; i < position.size(); ++i) {
      file << std::fixed << std::setprecision(4) << position[i] << ",";
    }
    file << std::endl;

    t += dt;
  }

  file.close();
  return true;
}