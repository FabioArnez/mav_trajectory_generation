#include "mav_trajectory_generation/example/example_planner_4dof.h"
#include "mav_trajectory_generation/polynomial_optimization_nonlinear.h"
#include <fstream>
#include <iomanip>

ExamplePlanner4D::ExamplePlanner4D()
    : max_v_(2.0),
      max_a_(2.0),
      current_velocity_(Eigen::Vector4d::Zero()),
      current_pose_(Eigen::Affine3d::Identity()) {
}

// Method to set maximum speed.
void ExamplePlanner4D::setMaxSpeed(const double max_v) {
  max_v_ = max_v;
}

// Method to get maximum speed.
double ExamplePlanner4D::getMaxSpeed() const {
  return max_v_;
}

// Method to set maximum acceleration.
void ExamplePlanner4D::setMaxAcceleration(const double max_a) {
  max_a_ = max_a;
}

// Method to get maximum acceleration.
double ExamplePlanner4D::getMaxAcceleration() const {
  return max_a_;
}

// Method to set current pose.
void ExamplePlanner4D::setCurrentPose(const Eigen::Affine3d& pose) {
  current_pose_ = pose;
}

// Method to set current velocity.
void ExamplePlanner4D::setCurrentVelocity(const Eigen::Vector4d& velocity) {
  current_velocity_ = velocity;
}

// Plans a trajectory from the current position to the a goal position and velocity
bool ExamplePlanner4D::planTrajectory(const Eigen::VectorXd& goal_pos,
                                    const Eigen::VectorXd& goal_vel,
                                    mav_trajectory_generation::Trajectory* trajectory) {
  assert(trajectory);
  trajectory->clear();

  // 4 Dimensional trajectory => 3D position + yaw
  const int dimension = 4;
  bool success = false;

  Eigen::Vector4d start_pos_4d, start_vel_4d;
  start_pos_4d << current_pose_.translation(),
      Eigen::Quaterniond(current_pose_.rotation()).toRotationMatrix().eulerAngles(2, 1, 0)[0];
  start_vel_4d << current_velocity_;

  success = planTrajectory(
      goal_pos, goal_vel, start_pos_4d, start_vel_4d, max_v_, max_a_,
      &(*trajectory));

  return success;
}

// Plans a trajectory from a start position and velocity to a goal position and velocity
bool ExamplePlanner4D::planTrajectory(const Eigen::VectorXd& goal_pos,
                                    const Eigen::VectorXd& goal_vel,
                                    const Eigen::VectorXd& start_pos,
                                    const Eigen::VectorXd& start_vel,
                                    double v_max, double a_max,
                                    mav_trajectory_generation::Trajectory* trajectory) {
  assert(trajectory);
  const int dimension = 4; // 4D trajectory (3D position + yaw)
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

bool ExamplePlanner4D::saveTrajectoryToCSV(const mav_trajectory_generation::Trajectory& trajectory,
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