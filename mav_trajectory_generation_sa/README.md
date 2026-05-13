# Description

The goal of this repository sub-folder is to provide a standalone package for the MAV [Trajectory Generation Package](https://github.com/ethz-asl/mav_trajectory_generation/tree/master#) (i.e., without ROS dependency). The idea is to incorporate the trajectory generation functionality into simulation environments such as [gym_pybullet_drones](https://utiasdsl.github.io/gym-pybullet-drones/), similar to what has been done before for the (deprecated) [AirSim Drone Racing Lab](https://github.com/microsoft/AirSim-Drone-Racing-Lab) simulator.


# Installation

The `CMakeLists.txt` auto-detects the active Python environment (`$CONDA_PREFIX` or `$VIRTUAL_ENV`), so no manual cmake flags are required. See the platform-specific sections below for dependency installation.

---

## macOS — Conda

### Create a Conda environment

```bash
conda create -n <env_name> python=3.10
conda activate <env_name>
```

### Install required packages

```bash
conda install -c conda-forge cxx-compiler
conda install conda-forge::cmake
conda install conda-forge::pkgconfig
conda install conda-forge::glog
conda install anaconda::gflags
conda install conda-forge::eigen
conda install -c conda-forge boost-cpp
conda install -c conda-forge yaml-cpp
conda install -c conda-forge gtest
conda install -c conda-forge nlopt
conda install -c conda-forge pybind11
```

### Build and install

```bash
mkdir build && cd build
cmake ..
make -j4
make install
```

---

## Linux (CachyOS / Arch) — UV

### Install C++ dependencies via pacman

```bash
sudo pacman -S --needed \
  cmake pkgconf \
  google-glog gflags \
  eigen boost yaml-cpp \
  gtest nlopt pybind11
```

### Create and activate a UV environment

```bash
uv venv .venv --python 3.10
source .venv/bin/activate
```

### Build and install

```bash
mkdir build && cd build
cmake ..
make -j$(nproc)
make install
```

---

# Build Trajectory Planner Code from Scratch

These steps describe how the standalone folder was originally assembled from upstream sources, for reference if rebuilding from scratch.

1. Download the repository code:
    ```bash
    git clone git@github.com:ethz-asl/mav_trajectory_generation.git
    ```

2. Move inside the `mav_trajectory_generation` subfolder, which corresponds to the trajectory planner code.

3. Download the files `eigen_mav_msgs.h` and `common.h` from the `mav_msgs` [repository](https://github.com/ethz-asl/mav_comm/tree/master/mav_msgs/include/mav_msgs), and put the files inside the `mav_trajectory_generation/include/mav_trajectory_generation` folder.
    - Rename `common.h` to `common_mav_msgs.h` for clarity.

4. Remove ROS dependencies by commenting out the following methods inside `common_mav_msgs.h`:
    - `vector3FromMsg(const geometry_msgs::Vector3& msg)`
    - `vector3FromPointMsg(const geometry_msgs::Point& msg)`
    - `quaternionFromMsg(const geometry_msgs::Quaternion& msg)`
    - `vectorEigenToMsg(const Eigen::Vector3d& eigen, geometry_msgs::Vector3* msg)`
    - `pointEigenToMsg(const Eigen::Vector3d& eigen, geometry_msgs::Point* msg)`
    - `quaternionEigenToMsg(const Eigen::Quaterniond& eigen, geometry_msgs::Quaternion* msg)`
    - `setQuaternionMsgFromYaw(double yaw, geometry_msgs::Quaternion* msg)`
    - `setAngularVelocityMsgFromYawRate(double yaw_rate, geometry_msgs::Vector3* msg)`

5. Copy the code from the `eigen-checks` [repository](https://github.com/ethz-asl/eigen_checks/tree/master) into the `include` and `src` folders respectively, i.e., copy the eigen-checks `include/eigen-checks` folder to `include`, and the eigen-checks `src` folder to `src`.

6. In the test files `test_polynomial_optimization.cpp` and `test_polynomial.cpp`, update the include header file paths:
    ```c++
    #include "mav_trajectory_generation/eigen-checks/entrypoint.h"
    #include "mav_trajectory_generation/eigen-checks/glog.h"
    #include "mav_trajectory_generation/eigen-checks/gtest.h"
    ```

7. In `test_polynomial_optimization.cpp`, replace deprecated macro calls:
    ```c++
    // Replace:
    INSTANTIATE_TEST_CASE_P(OneDimension, PolynomialOptimizationTests, ...);
    INSTANTIATE_TEST_CASE_P(ThreeDimensions, PolynomialOptimizationTests, ...);
    INSTANTIATE_TEST_CASE_P(Derivatives, PolynomialOptimizationTests, ...);

    // With:
    INSTANTIATE_TEST_SUITE_P(OneDimension, PolynomialOptimizationTests, ...);
    INSTANTIATE_TEST_SUITE_P(ThreeDimensions, PolynomialOptimizationTests, ...);
    INSTANTIATE_TEST_SUITE_P(Derivatives, PolynomialOptimizationTests, ...);
    ```

8. Compile using the steps in the platform-specific Installation sections above.
