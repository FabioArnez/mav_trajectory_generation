cmake_minimum_required(VERSION 3.10...3.28)
project(mav_trajectory_generation)

# macOS: only force arm64 when actually running on Apple Silicon
if(APPLE)
  execute_process(COMMAND uname -m OUTPUT_VARIABLE HOST_ARCH OUTPUT_STRIP_TRAILING_WHITESPACE)
  if(HOST_ARCH STREQUAL "arm64")
    set(CMAKE_OSX_ARCHITECTURES "arm64" CACHE STRING "Build for arm64" FORCE)
  endif()
endif()

include_directories(include)

# Auto-detect active Python environment: Conda > UV/venv > system
# Override with: cmake -DENV_PREFIX=/path/to/env ..
if(NOT DEFINED ENV_PREFIX OR ENV_PREFIX STREQUAL "")
  if(DEFINED ENV{CONDA_PREFIX} AND NOT "$ENV{CONDA_PREFIX}" STREQUAL "")
    set(ENV_PREFIX $ENV{CONDA_PREFIX})
    message(STATUS "Detected Conda environment: ${ENV_PREFIX}")
  elseif(DEFINED ENV{VIRTUAL_ENV} AND NOT "$ENV{VIRTUAL_ENV}" STREQUAL "")
    set(ENV_PREFIX $ENV{VIRTUAL_ENV})
    message(STATUS "Detected UV/venv environment: ${ENV_PREFIX}")
  else()
    set(ENV_PREFIX "")
    message(STATUS "No Python environment detected, using system paths")
  endif()
endif()

# Library search hints: env prefix first, system paths as fallback
set(ENV_LIB_HINT "")
set(ENV_INC_HINT "")
if(NOT ENV_PREFIX STREQUAL "")
  set(ENV_LIB_HINT ${ENV_PREFIX}/lib)
  set(ENV_INC_HINT ${ENV_PREFIX}/include)
endif()

if(NOT Python_EXECUTABLE AND NOT ENV_PREFIX STREQUAL "")
  set(Python_EXECUTABLE ${ENV_PREFIX}/bin/python)
endif()

find_package(Python REQUIRED COMPONENTS Interpreter Development)
message(STATUS "Using Python: ${Python_EXECUTABLE}")
message(STATUS "Using Python Libraries: ${Python_LIBRARIES}")

# Add Python include directory
include_directories(${Python_INCLUDE_DIRS})

# Install into the active environment root by default
if(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT AND NOT ENV_PREFIX STREQUAL "")
  set(CMAKE_INSTALL_PREFIX "${ENV_PREFIX}" CACHE PATH "Install prefix" FORCE)
endif()

# Get Python version components
execute_process(
  COMMAND ${Python_EXECUTABLE} -c "import sys; print(sys.version_info.major)"
  OUTPUT_VARIABLE PYTHON_VERSION_MAJOR
  OUTPUT_STRIP_TRAILING_WHITESPACE
)

execute_process(
  COMMAND ${Python_EXECUTABLE} -c "import sys; print(sys.version_info.minor)"
  OUTPUT_VARIABLE PYTHON_VERSION_MINOR
  OUTPUT_STRIP_TRAILING_WHITESPACE
)

message(STATUS "Using Python: ${Python_EXECUTABLE}")
message(STATUS "Python version: ${PYTHON_VERSION_MAJOR}.${PYTHON_VERSION_MINOR}")

# Find and include glog library.
find_library(glog_LIBRARY NAMES glog HINTS ${ENV_LIB_HINT})
find_path(glog_INCLUDE_DIR NAMES glog/logging.h HINTS ${ENV_INC_HINT})

set(glog_INCLUDE_DIRS ${glog_INCLUDE_DIR})
set(glog_LIBRARIES ${glog_LIBRARY})

include_directories(${glog_INCLUDE_DIRS})

message(STATUS "glog_INCLUDE_DIRS: ${glog_INCLUDE_DIRS}")
message(STATUS "glog_LIBRARIES: ${glog_LIBRARIES}")

# Find and include Boost library.
# boost::system is header-only since Boost 1.71, so no component needed.
find_package(Boost REQUIRED)
include_directories(${Boost_INCLUDE_DIRS})

set(CMAKE_MACOSX_RPATH 0)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Link against system yaml-cpp if installed.
find_package(PkgConfig)

find_library(YamlCpp_LIBRARY NAMES yaml-cpp HINTS ${ENV_LIB_HINT})
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
find_library(NLopt_LIBRARY NAMES nlopt HINTS ${ENV_LIB_HINT})
find_path(NLopt_INCLUDE_DIR NAMES nlopt.hpp HINTS ${ENV_INC_HINT})

# Find and include pybind11
find_package(pybind11 REQUIRED)
# find_library(pybind11_LIBRARY NAMES pybind11 PATHS ${CONDA_PREFIX}/lib)
# find_path(pybind11_INCLUDE_DIR NAMES pybind11/pybind11.h PATHS ${CONDA_PREFIX}/include)

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
  src/example/example_planner_4dof.cc
)

# Link against yaml-cpp and glog.
target_link_libraries(${PROJECT_NAME} ${YamlCpp_LIBRARY} ${glog_LIBRARIES} Eigen3::Eigen ${NLopt_LIBRARY})

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

###################
# PYTHON BINDINGS #
###################

# Add Python bindings executable
pybind11_add_module(trajectory_planner
  src/pybindings/trajectory_planner_4dof_pybindings.cc
  src/pybindings/trajectory_planner_4dof.cc
  src/motion_defines.cpp
  src/polynomial.cpp
  src/polynomial_timing_evaluation.cpp
  src/motion_defines.cpp
  src/trajectory.cpp
  src/trajectory_sampling.cpp
  src/vertex.cpp
  src/timing.cpp
  src/segment.cpp
  src/io.cpp
  src/rpoly/rpoly_ak1.cpp
)

# Add this line to link the Python module to its required libraries
target_link_libraries(trajectory_planner PRIVATE ${PROJECT_NAME} ${glog_LIBRARIES} ${NLopt_LIBRARY} ${YamlCpp_LIBRARY} Eigen3::Eigen)

# Set the output name and install directory
set_target_properties(trajectory_planner PROPERTIES
  PREFIX ""
  SUFFIX ".so"
  OUTPUT_NAME "trajectory_planner"
)

# -undefined dynamic_lookup is a macOS-only linker flag
if(APPLE)
  set_target_properties(trajectory_planner PROPERTIES
    LINK_FLAGS "-undefined dynamic_lookup"
  )
endif()

# Install the Python module
install(TARGETS trajectory_planner
  LIBRARY DESTINATION lib/python${PYTHON_VERSION_MAJOR}.${PYTHON_VERSION_MINOR}/site-packages
)

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
# install(TARGETS ${PROJECT_NAME} polynomial_timing_evaluation
#   ARCHIVE DESTINATION lib
#   LIBRARY DESTINATION lib
#   RUNTIME DESTINATION bin
# )