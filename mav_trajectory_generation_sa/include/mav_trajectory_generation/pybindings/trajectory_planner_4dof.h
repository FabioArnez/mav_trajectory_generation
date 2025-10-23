#ifndef MAV_TRAJECTORY_GENERATION_TRAJECTORY_PLANNER_4DOF_H
#define MAV_TRAJECTORY_GENERATION_TRAJECTORY_PLANNER_4DOF_H

#include <iostream>
#include <Eigen/Dense>
#include "mav_trajectory_generation/polynomial_optimization_nonlinear.h"

class TrajectoryPlanner4DOF {
    public:
        TrajectoryPlanner4DOF();

        void setMaxSpeed(double max_v);
        double getMaxSpeed() const;

        void setMaxAcceleration(double max_a);
        double getMaxAcceleration() const;

        void setCurrentPose(const Eigen::Affine3d& pose);
        void setCurrentVelocity(const Eigen::Vector4d& velocity);

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
        Eigen::Vector4d current_velocity_;
        double max_v_; // m/s
        double max_a_; // m/s^2
};

#endif // MAV_TRAJECTORY_GENERATION_TRAJECTORY_PLANNER_4DOF_H