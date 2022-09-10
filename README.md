# ROS2_CONTROLLERS FOR SOLO 12

## Overview

This repository aims at testing controllers on a quadruped robot, [SOLO 12](https://github.com/open-dynamic-robot-initiative/open_robot_actuator_hardware/blob/master/mechanics/quadruped_robot_12dof_v1/README.md), in the
Gazebo simulator. We will eventually run these controllers on the real robot, but this functionality has not been implemented yet. Furthermore, this repository relies only on the effort interface from ROS2_control.

## Installations
Here are the dependencies used.
- ros2-control
- ros2-controllers
- ros-foxy-gazebo-ros-pkgs
- ros-foxy-xacro
- ros-foxy-gazebo-ros2-control
- ros-foxy-realtime-tools
- ros-foxy-angles
- odri-control-interface and master-board directories from this [link](https://github.com/open-dynamic-robot-initiative/odri_control_interface). These packages and the ones listed below should go in the `src` folder of your workspace.
- mpi_cmake_modules from [here](https://github.com/machines-in-motion/mpi_cmake_modules)
- pybind11 from [here](https://github.com/pybind/pybind11)
- ros2_control_solo from [here](https://github.com/stack-of-tasks/ros2_control_solo)

You can also run `rosdep install --from-paths src --ignore-src -r -y` to install everything in one command, but it might miss a dependency (as I might not have listed all dependencies). 
Once the dependencies are installed, build the workspace with `colcon build --symlink-install`. Note that if building the workspace consumes too much memory, you can try instead
`MAKEFLAGS="-j1 -l1" colcon build --symlink-install --executor sequential`. **Since we use `--symlink-install`, you will also need to 
`chmod +x ros2_controllers_test_nodes/ros2_control_test_nodes/test_controllers.py`, in order for one of the launch files to execute this node.** 
## Running the simulation

The following commands runs a PD controller on SOLO in Gazebo using the implementation of the ROS node in Python. **Note that before running the simulation, 
in the package ros2_description_solo (from ros2_control_solo), on line 1031 of the file urdf/solo12_simulation.sdf, please write your absolute path to the file src/ros2_control_solo_bringup/config/solo_gazebo_test_controllers.yaml (in the ros2_control_solo_bringup package)** (as I have not figured out how to dynamically determine the absolute path of a file in an SDF file). 
1. Source ROS2 foxy and source your install file in the workspace
2. `source /usr/share/gazebo/setup.sh` (since some Gazebo environment variables will be overridden)
2. In your workspace directory, `ros2 launch ros2_control_solo_bringup solo_system_position_only_gazebo.launch.py` launches the Gazebo simulation with SOLO.
3. `ros2 launch ros2_controllers_solo_bringup solo_test_controllers.launch.py` starts the test_controllers node while leaving the simulation paused.
4. `ros2 service call trigger_PD std_srvs/srv/Trigger` starts the PD controller.

To let SOLO start walking (which might not be working since I am currently working on it), you can unpause the simulation and run `ros2 service call trigger_walk std_srvs/srv/Trigger`.

## Repository Description

[ros2_controllers_solo_bringup](ros2_controllers_solo_bringup): composed of launch files launching the controllers and yaml files storing parameters.
- `solo_test_controllers.launch.py` launches a ROS2 node implemented in Python running controllers.
- `solo_test_controllers_cpp.launch.py` launches a ROS2 node implemented in C++ running controllers.
- Also in the config folder, `solo_test_controllers.yaml` is called by `solo_test_controllers.launch.py` and `solo_test_controllers_cpp.launch.py`. This yaml file ccontains parameters for the controllers.

[ros2_control_test_nodes](ros2_control_test_nodes): contains the implementation of the controllers.
- C++ code is in the src and include subdirectories.
- The Python code is in the ros2_control_test_nodes subdirectory.
  - `test_controllers.py` is the main script that runs the ROS node, which itself is defined in `node.py`. `node.py` has all the controllers implemented as classes. 
- Most subdirectories and files inside the subdirectories listed above are obtained from the [mim_control](https://github.com/machines-in-motion/mim_control) repository,
the [robot_properties_solo](https://github.com/open-dynamic-robot-initiative/robot_properties_solo) repository, and the [reactive_planners](https://github.com/machines-in-motion/reactive_planners) repository (with modifications).
- Some C++ files are currently a work in progress, so they may not work.

## Notes

- The ros2_control_test_nodes package has a lot of files for different controllers. It might be better to create an individual package for each controller and language (Python and C++).
- package.xml and README.md files for individual packages need to be updated.
