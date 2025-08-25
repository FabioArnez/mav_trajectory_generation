#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include "mav_trajectory_generation/example/example_planner_4dof.h"
#include "mav_trajectory_generation/polynomial_optimization_nonlinear.h"
#include "mav_trajectory_generation/motion_defines.h"

namespace py = pybind11;
using namespace py::literals;
using namespace mav_trajectory_generation;

PYBIND11_MODULE(trajectory_planner, m) {
    m.doc() = "Python bindings for the 4D trajectory planner";

    // Bind ExamplePlanner4D class
    py::class_<ExamplePlanner4D>(m, "ExamplePlanner4D")
        .def(py::init<>())
        .def("setMaxSpeed", &ExamplePlanner4D::setMaxSpeed)
        .def("getMaxSpeed", &ExamplePlanner4D::getMaxSpeed)
        .def("setMaxAcceleration", &ExamplePlanner4D::setMaxAcceleration)
        .def("getMaxAcceleration", &ExamplePlanner4D::getMaxAcceleration)
        .def("setCurrentPose", &ExamplePlanner4D::setCurrentPose)
        .def("setCurrentVelocity", &ExamplePlanner4D::setCurrentVelocity)
        .def("planTrajectory", py::overload_cast<const Eigen::VectorXd&, const Eigen::VectorXd&, Trajectory*>(&ExamplePlanner4D::planTrajectory))
        .def("planTrajectory", py::overload_cast<const Eigen::VectorXd&, const Eigen::VectorXd&, const Eigen::VectorXd&, const Eigen::VectorXd&, double, double, Trajectory*>(&ExamplePlanner4D::planTrajectory))
        .def("saveTrajectoryToCSV", &ExamplePlanner4D::saveTrajectoryToCSV);

    // Bind Trajectory class
    py::class_<Trajectory>(m, "Trajectory")
        .def(py::init<>())
        .def("clear", &Trajectory::clear)
        .def("getMaxTime", &Trajectory::getMaxTime)
        .def("evaluate", &Trajectory::evaluate)
        .def("scaleSegmentTimesToMeetConstraints", &Trajectory::scaleSegmentTimesToMeetConstraints);

    // Bind Vertex class
    py::class_<Vertex>(m, "Vertex")
        .def(py::init<size_t>())
        .def("makeStartOrEnd", py::overload_cast<const Eigen::VectorXd&, int>(&Vertex::makeStartOrEnd))
        .def("addConstraint", py::overload_cast<int, const Eigen::VectorXd&>(&Vertex::addConstraint));

    // Bind NonlinearOptimizationParameters struct
    py::class_<NonlinearOptimizationParameters>(m, "NonlinearOptimizationParameters")
        .def(py::init<>())
        .def_readwrite("f_abs", &NonlinearOptimizationParameters::f_abs)
        .def_readwrite("f_rel", &NonlinearOptimizationParameters::f_rel)
        .def_readwrite("x_rel", &NonlinearOptimizationParameters::x_rel)
        .def_readwrite("x_abs", &NonlinearOptimizationParameters::x_abs)
        .def_readwrite("initial_stepsize_rel", &NonlinearOptimizationParameters::initial_stepsize_rel)
        .def_readwrite("equality_constraint_tolerance", &NonlinearOptimizationParameters::equality_constraint_tolerance)
        .def_readwrite("inequality_constraint_tolerance", &NonlinearOptimizationParameters::inequality_constraint_tolerance)
        .def_readwrite("max_iterations", &NonlinearOptimizationParameters::max_iterations)
        .def_readwrite("time_penalty", &NonlinearOptimizationParameters::time_penalty)
        .def_readwrite("algorithm", &NonlinearOptimizationParameters::algorithm)
        .def_readwrite("random_seed", &NonlinearOptimizationParameters::random_seed)
        .def_readwrite("use_soft_constraints", &NonlinearOptimizationParameters::use_soft_constraints)
        .def_readwrite("soft_constraint_weight", &NonlinearOptimizationParameters::soft_constraint_weight)
        .def_readwrite("time_alloc_method", &NonlinearOptimizationParameters::time_alloc_method)
        .def_readwrite("print_debug_info", &NonlinearOptimizationParameters::print_debug_info)
        .def_readwrite("print_debug_info_time_allocation", &NonlinearOptimizationParameters::print_debug_info_time_allocation);

    // Bind PolynomialOptimizationNonLinear class
    py::class_<PolynomialOptimizationNonLinear<>>(m, "PolynomialOptimizationNonLinear")
        .def(py::init<size_t, const NonlinearOptimizationParameters&>())
        .def("setupFromVertices", &PolynomialOptimizationNonLinear<>::setupFromVertices)
        .def("addMaximumMagnitudeConstraint", &PolynomialOptimizationNonLinear<>::addMaximumMagnitudeConstraint)
        .def("optimize", &PolynomialOptimizationNonLinear<>::optimize)
        .def("getTrajectory", &PolynomialOptimizationNonLinear<>::getTrajectory);

    // Bind derivative_order constants
    m.attr("derivative_order") = py::dict(
        "POSITION"_a = mav_trajectory_generation::derivative_order::POSITION,
        "VELOCITY"_a = mav_trajectory_generation::derivative_order::VELOCITY,
        "ACCELERATION"_a = mav_trajectory_generation::derivative_order::ACCELERATION,
        "JERK"_a = mav_trajectory_generation::derivative_order::JERK,
        "SNAP"_a = mav_trajectory_generation::derivative_order::SNAP,
        "ORIENTATION"_a = mav_trajectory_generation::derivative_order::ORIENTATION,
        "ANGULAR_VELOCITY"_a = mav_trajectory_generation::derivative_order::ANGULAR_VELOCITY,
        "ANGULAR_ACCELERATION"_a = mav_trajectory_generation::derivative_order::ANGULAR_ACCELERATION,
        "INVALID"_a = mav_trajectory_generation::derivative_order::INVALID
    );

    // Bind nlopt::algorithm enum
    py::enum_<nlopt::algorithm>(m, "nlopt_algorithm")
        .value("LN_COBYLA", nlopt::algorithm::LN_COBYLA)
        .value("LN_BOBYQA", nlopt::algorithm::LN_BOBYQA)
        .value("LN_NEWUOA", nlopt::algorithm::LN_NEWUOA)
        .value("LN_NEWUOA_BOUND", nlopt::algorithm::LN_NEWUOA_BOUND)
        .value("LN_PRAXIS", nlopt::algorithm::LN_PRAXIS)
        .value("LN_NELDERMEAD", nlopt::algorithm::LN_NELDERMEAD)
        .value("LN_SBPLX", nlopt::algorithm::LN_SBPLX)
        .value("GN_CRS2_LM", nlopt::algorithm::GN_CRS2_LM)
        .value("GN_MLSL", nlopt::algorithm::GN_MLSL)
        .value("GN_MLSL_LDS", nlopt::algorithm::GN_MLSL_LDS)
        .value("GN_DIRECT", nlopt::algorithm::GN_DIRECT)
        .value("GN_DIRECT_L", nlopt::algorithm::GN_DIRECT_L)
        .value("GN_DIRECT_L_RAND", nlopt::algorithm::GN_DIRECT_L_RAND)
        .value("GN_DIRECT_NOSCAL", nlopt::algorithm::GN_DIRECT_NOSCAL)
        .value("GN_DIRECT_L_NOSCAL", nlopt::algorithm::GN_DIRECT_L_NOSCAL)
        .value("GN_DIRECT_L_RAND_NOSCAL", nlopt::algorithm::GN_DIRECT_L_RAND_NOSCAL)
        .value("GN_ORIG_DIRECT", nlopt::algorithm::GN_ORIG_DIRECT)
        .value("GN_ORIG_DIRECT_L", nlopt::algorithm::GN_ORIG_DIRECT_L)
        .value("GD_STOGO", nlopt::algorithm::GD_STOGO)
        .value("GD_STOGO_RAND", nlopt::algorithm::GD_STOGO_RAND)
        .value("LD_LBFGS", nlopt::algorithm::LD_LBFGS)
        .value("LD_VAR1", nlopt::algorithm::LD_VAR1)
        .value("LD_VAR2", nlopt::algorithm::LD_VAR2)
        .value("LD_TNEWTON", nlopt::algorithm::LD_TNEWTON)
        .value("LD_TNEWTON_RESTART", nlopt::algorithm::LD_TNEWTON_RESTART)
        .value("LD_TNEWTON_PRECOND", nlopt::algorithm::LD_TNEWTON_PRECOND)
        .value("LD_TNEWTON_PRECOND_RESTART", nlopt::algorithm::LD_TNEWTON_PRECOND_RESTART)
        .value("GN_CRS2_LM", nlopt::algorithm::GN_CRS2_LM)
        .value("GN_ESCH", nlopt::algorithm::GN_ESCH)
        .value("GN_ISRES", nlopt::algorithm::GN_ISRES)
        .value("AUGLAG", nlopt::algorithm::AUGLAG)
        .value("AUGLAG_EQ", nlopt::algorithm::AUGLAG_EQ)
        .value("G_MLSL", nlopt::algorithm::G_MLSL)
        .value("G_MLSL_LDS", nlopt::algorithm::G_MLSL_LDS)
        .value("LD_MMA", nlopt::algorithm::LD_MMA)
        .value("LD_SLSQP", nlopt::algorithm::LD_SLSQP)
        .value("LD_CCSAQ", nlopt::algorithm::LD_CCSAQ)
        .value("GN_AGS", nlopt::algorithm::GN_AGS)
        .value("GN_CRS2_LM", nlopt::algorithm::GN_CRS2_LM)
        .value("GN_ESCH", nlopt::algorithm::GN_ESCH)
        .value("GN_MLSL", nlopt::algorithm::GN_MLSL)
        .value("GN_MLSL_LDS", nlopt::algorithm::GN_MLSL_LDS)
        .value("GN_ORIG_DIRECT", nlopt::algorithm::GN_ORIG_DIRECT)
        .value("GN_ORIG_DIRECT_L", nlopt::algorithm::GN_ORIG_DIRECT_L)
        .value("LD_AUGLAG", nlopt::algorithm::LD_AUGLAG)
        .value("LD_AUGLAG_EQ", nlopt::algorithm::LD_AUGLAG_EQ)
        .value("LD_CCSAQ", nlopt::algorithm::LD_CCSAQ)
        .value("LD_LBFGS", nlopt::algorithm::LD_LBFGS)
        .value("LD_MMA", nlopt::algorithm::LD_MMA)
        .value("LD_SLSQP", nlopt::algorithm::LD_SLSQP)
        .value("LD_TNEWTON", nlopt::algorithm::LD_TNEWTON)
        .value("LD_TNEWTON_PRECOND", nlopt::algorithm::LD_TNEWTON_PRECOND)
        .value("LD_TNEWTON_PRECOND_RESTART", nlopt::algorithm::LD_TNEWTON_PRECOND_RESTART)
        .value("LD_TNEWTON_RESTART", nlopt::algorithm::LD_TNEWTON_RESTART)
        .value("LD_VAR1", nlopt::algorithm::LD_VAR1)
        .value("LD_VAR2", nlopt::algorithm::LD_VAR2)
        .value("LN_SBPLX", nlopt::algorithm::LN_SBPLX)
        .value("LN_BOBYQA", nlopt::algorithm::LN_BOBYQA)
        .value("LN_COBYLA", nlopt::algorithm::LN_COBYLA)
        .value("LN_NEWUOA", nlopt::algorithm::LN_NEWUOA)
        .value("LN_NEWUOA_BOUND", nlopt::algorithm::LN_NEWUOA_BOUND)
        .value("LN_NELDERMEAD", nlopt::algorithm::LN_NELDERMEAD)
        .value("LN_PRAXIS", nlopt::algorithm::LN_PRAXIS)
        .value("LN_SBPLX", nlopt::algorithm::LN_SBPLX);
}