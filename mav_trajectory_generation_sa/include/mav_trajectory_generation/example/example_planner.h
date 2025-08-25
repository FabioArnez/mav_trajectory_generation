#ifndef MAV_TRAJECTORY_GENERATION_EXAMPLE_PLANNER_H
#define MAV_TRAJECTORY_GENERATION_EXAMPLE_PLANNER_H

#include <iostream>
#include <Eigen/Dense>
#include "mav_trajectory_generation/polynomial_optimization_nonlinear.h"

class ExamplePlanner{
    public:
        ExamplePlanner();

        void setMaxSpeed(double max_v);
        double getMaxSpeed() const;

        void setMaxAcceleration(double max_a);
        double getMaxAcceleration() const;

        void setMaxAngularSpeed(double max_ang_v);
        double getMaxAngularSpeed() const;

        void setMaxAngularAcceleration(double max_ang_a);
        double getMaxAngularAcceleration() const;

        void setCurrentPose(const Eigen::Affine3d& pose);
        void setCurrentVelocity(const Eigen::Vector3d& velocity);
        void setCurrentAngularVelocity(const Eigen::Vector3d& angular_velocity);

        bool planTrajectory(const Eigen::VectorXd& goal_pos,
                            const Eigen::VectorXd& goal_vel,
                            mav_trajectory_generation::Trajectory* trajectory);

        bool planTrajectory(const Eigen::VectorXd& goal_pos,
                            const Eigen::VectorXd& goal_vel,
                            const Eigen::VectorXd& start_pos,
                            const Eigen::VectorXd& start_vel,
                            double v_max, double a_max,
                            mav_trajectory_generation::Trajectory* trajectory);

        bool saveTrajectoryToCSV(const mav_trajectory_generation::Trajectory& trajectory,
                                 const std::string& filename);

    private:
        Eigen::Affine3d current_pose_;
        Eigen::Vector3d current_velocity_;
        Eigen::Vector3d current_angular_velocity_;
        double max_v_; // m/s
        double max_a_; // m/s^2
        double max_ang_v_;
        double max_ang_a_;
};

#endif // MAV_TRAJECTORY_GENERATION_EXAMPLE_PLANNER_H