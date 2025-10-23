# Description

The goal of this repository sub-folder is to provide an standalone package for the MAV [Trajectory Generation Package](https://github.com/ethz-asl/mav_trajectory_generation/tree/master#) (i.e., without ROS dependecy). The idea is to incorporate the trajectory generation functionality to simualtion environments such as [gym_pybullet_drones](https://utiasdsl.github.io/gym-pybullet-drones/), similar to what has be done before for the (deprecated) [AirSim Drone Racing Lab](https://github.com/microsoft/AirSim-Drone-Racing-Lab) simulator.


# Installation

## Create a conda environment
 For this step, simply use the conda environment proposed by the [gym-pybullet-drones GitHub](https://github.com/utiasDSL/gym-pybullet-drones) repository.

## Install Required Packages

Install the following dependencies in the conda environment, for succesful C++ compilation.

```bash
conda install conda-forge::cmake
conda install conda-forge::pkgconfig
conda install conda-forge::glog
conda install anaconda::gflags
conda install conda-forge::eigen
conda install -c conda-forge boost-cpp
conda install -c conda-forge yaml-cpp
conda install -c conda-forge gtest==1.16.0
conda install -c conda-forge nlopt==2.8.0
```

For Python bindings support use:
```bash
conda install -c conda-forge pybind11
```

## Build Trajectory Planner Code

1. Download the repository code:
    ```bash
    git clone git@github.com:ethz-asl/mav_trajectory_generation.git
    ```

2. Move inside the `mav_trajectory_generation` subfolder, which corresponds to the trajectory planner code.

3. Download the files `eigen_mav_msgs.h` and `common.h` from the `mav_msgs` [repository](https://github.com/ethz-asl/mav_comm/tree/master/mav_msgs/include/mav_msgs), and put the files inside the `mav_trajectory_generation/include/mav_trajectory_generation` folder
	- Rename `common.h` to `common_mav_msgs.h` for clarity.

4. Remove ROS dependencies by commenting out the following methods inside the `common_mav_msgs.h` file:
	- `vector3FromMsg(const geometry_msgs::Vector3& msg)`
	- `vector3FromPointMsg(const geometry_msgs::Point& msg)`
	- `quaternionFromMsg(const geometry_msgs::Quaternion& msg)`
	- `vectorEigenToMsg(const Eigen::Vector3d& eigen, geometry_msgs::Vector3* msg)`
	- `pointEigenToMsg(const Eigen::Vector3d& eigen, geometry_msgs::Point* msg)`
	- `quaternionEigenToMsg(const Eigen::Quaterniond& eigen, geometry_msgs::Quaternion* msg)`
	- `setQuaternionMsgFromYaw(double yaw, geometry_msgs::Quaternion* msg)`
	- `setAngularVelocityMsgFromYawRate(double yaw_rate, geometry_msgs::Vector3* msg)`

5. Copy the code from the `eigen-checks` [repository](https://github.com/ethz-asl/eigen_checks/tree/master) into the `include` and `src` folder, respectively, i.e., copy the eigen-checks `include/eigen-checks` folder to the `include` folder, and the eigen-checks `src` folder to the `src` folder.

6. In the test files  `test_polynomial_optimization.cpp` and  `test_polynomial.cpp`  update the include header file paths:
    ```c++
    #include "mav_trajectory_generation/eigen-checks/entrypoint.h"
    #include "mav_trajectory_generation/eigen-checks/glog.h"
    #include "mav_trajectory_generation/eigen-checks/gtest.h"
    ```

7.  In the test files `test_polynomial_optimization.cpp`, change the following deprecated code:
    ```C++
    // Replace these deprecated calls:
    INSTANTIATE_TEST_CASE_P(OneDimension, PolynomialOptimizationTests, ...);
    INSTANTIATE_TEST_CASE_P(ThreeDimensions, PolynomialOptimizationTests, ...);
    INSTANTIATE_TEST_CASE_P(Derivatives, PolynomialOptimizationTests, ...);

    // With these updated calls:
    INSTANTIATE_TEST_SUITE_P(OneDimension, PolynomialOptimizationTests, ...);
    INSTANTIATE_TEST_SUITE_P(ThreeDimensions, PolynomialOptimizationTests, ...);
    INSTANTIATE_TEST_SUITE_P(Derivatives, PolynomialOptimizationTests, ...);
    ```
8. Compile the code using the `CMakeLists.txt` file:
    - Create a build directory `$mkdir build` and move inside `$cd build`.
    - Since we use a conda environment, we must use `cmake` prioritizing the conda environment libraries as suggested in this [link](https://wolfv.github.io/posts/2019/06/12/conda-for-cpp.html), and using the following command:
        ```bash
        cmake -DCONDA_PREFIX=$CONDA_PREFIX .. 
        ```

        For Python bindings, use the following command:

        ```bash
        cmake -DCONDA_PREFIX=$CONDA_PREFIX -DCMAKE_INSTALL_PREFIX=$CONDA_PREFIX ..   
        ```

    - Once this is done, finally run the following command:
    
        ```bash
        make
        ```
