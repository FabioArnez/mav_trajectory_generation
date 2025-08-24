cmake_minimum_required(VERSION 2.8.3...3.5)
project(mav_trajectory_generation)

include_directories(include)

set(CMAKE_PREFIX_PATH ${CONDA_PREFIX} ${CMAKE_PREFIX_PATH})

find_library(glog_LIBRARY NAMES glog PATHS ${CONDA_PREFIX}/lib)
find_path(glog_INCLUDE_DIR NAMES glog/logging.h PATHS ${CONDA_PREFIX}/include)

set(glog_INCLUDE_DIRS ${glog_INCLUDE_DIR})
set(glog_LIBRARIES ${glog_LIBRARY})

include_directories(${glog_INCLUDE_DIRS})

message(STATUS "glog_INCLUDE_DIRS: ${glog_INCLUDE_DIRS}")
message(STATUS "glog_LIBRARIES: ${glog_LIBRARIES}")

# Find and include Boost library.
find_package(Boost REQUIRED COMPONENTS system)
include_directories(${Boost_INCLUDE_DIRS})

set(CMAKE_MACOSX_RPATH 0)
add_definitions(-std=c++14)

# Link against system yaml-cpp if installed.
find_package(PkgConfig)

find_library(YamlCpp_LIBRARY NAMES yaml-cpp PATHS ${CONDA_PREFIX}/lib)
# find_package(yaml-cpp REQUIRED)
# pkg_check_modules(YamlCpp REQUIRED yaml-cpp>=0.5)

# Find and include Eigen library.
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIRS})

# Find and include gtest library.
find_package(GTest REQUIRED)
include_directories(${GTEST_INCLUDE_DIRS})

# Find and include NLopt library.
# find_package(NLopt REQUIRED)
# include_directories(${NLopt_INCLUDE_DIRS})
# Manually specify NLopt library path
find_library(NLopt_LIBRARY NAMES nlopt PATHS ${CONDA_PREFIX}/lib)
find_path(NLopt_INCLUDE_DIR NAMES nlopt.hpp PATHS ${CONDA_PREFIX}/include)

#############
# LIBRARIES #
#############
add_library(${PROJECT_NAME} STATIC
  src/motion_defines.cpp
  src/polynomial.cpp
  src/segment.cpp
  src/timing.cpp
  src/trajectory.cpp
  src/trajectory_sampling.cpp
  src/vertex.cpp
  src/io.cpp
  src/rpoly/rpoly_ak1.cpp
)

# Link against yaml-cpp and glog.
target_link_libraries(${PROJECT_NAME} ${YamlCpp_LIBRARY} ${glog_LIBRARIES} Eigen3::Eigen Boost::system ${NLopt_LIBRARY})

############
# BINARIES #
############
add_executable(polynomial_timing_evaluation
  src/polynomial_timing_evaluation.cpp
)
target_link_libraries(polynomial_timing_evaluation ${PROJECT_NAME})

add_executable(trajectory_generation_example
  src/example/example_planner.cc
  src/example/example_planner_node.cc
)
target_link_libraries(trajectory_generation_example ${PROJECT_NAME})

add_executable(trajectory_generation_6dof_example
        src/example/example_planner_6dof.cc
        src/example/example_planner_6dof_node.cc
)

target_link_libraries(trajectory_generation_6dof_example ${PROJECT_NAME})

add_executable(trajectory_generation_4dof_example
        src/example/example_planner_4dof.cc
        src/example/example_planner_4dof_node.cc
)

target_link_libraries(trajectory_generation_4dof_example ${PROJECT_NAME})

#########
# TESTS #
#########
enable_testing()
add_test(NAME test_polynomial COMMAND test_polynomial)
add_test(NAME test_polynomial_optimization COMMAND test_polynomial_optimization)

add_executable(test_polynomial
  test/test_polynomial.cpp
)
target_link_libraries(test_polynomial ${PROJECT_NAME} ${GTEST_LIBRARIES} ${GTEST_MAIN_LIBRARIES} ${NLopt_LIBRARY})

add_executable(test_polynomial_optimization
  test/test_polynomial_optimization.cpp
)
target_link_libraries(test_polynomial_optimization ${PROJECT_NAME} ${GTEST_LIBRARIES} ${GTEST_MAIN_LIBRARIES} ${NLopt_LIBRARY})

##########
# INSTALL #
##########
install(TARGETS ${PROJECT_NAME} polynomial_timing_evaluation
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)