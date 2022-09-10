cmake_minimum_required(VERSION 3.8)
project(ros2_control_test_nodes)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()


find_package(PkgConfig)        # find pkg_config

# find dependencies
# Build
find_package(mpi_cmake_modules REQUIRED)

# External
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(urdf REQUIRED)
find_package(xacro REQUIRED)
find_package(control_msgs REQUIRED)
find_package(rclpy REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(eiquadprog REQUIRED)
find_package(eigen-quadprog REQUIRED)
find_package(pinocchio REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(pybind11 REQUIRED)
find_package(Python ${PYTHON_VERSION_STRING} REQUIRED)
if(NOT ${Python_FOUND})
    message(FATAL_ERROR "PYTHON not FOUND by FindPython")
endif()
search_for_boost_python()
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

include_directories(
        include
        ${Eigen3_INCLUDE_DIRS}
        ${PINOCCHIO_INCLUDE_DIRS}
)

# https://answers.ros.org/question/298740/ros2-c-and-python-in-same-package-is-it-possible/
# C++ executables
add_executable(test_controllers_cpp
        src/TestControllers.cpp
        src/ControllersNode.cpp
        src/Algorithms/PD_control/PD_control.cpp
        src/Algorithms/mim_control/centroidal_force_qp_controller.cpp
        src/Algorithms/mim_control/centroidal_pd_controller.cpp
        src/Algorithms/mim_control/impedance_controller.cpp
        src/Algorithms/mim_control/demo_com_ctrl_cpp.cpp
        src/Algorithms/reactive_planners/demo_reactive_planners_solo12_step_adjustment.cpp
        src/Algorithms/reactive_planners/quadruped_dcm_reactive_stepper.cpp
        src/Algorithms/reactive_planners/dcm_reactive_stepper.cpp
        src/Algorithms/reactive_planners/dcm_vrp_planner.cpp
        src/Algorithms/reactive_planners/dynamically_consistent_end_effector_trajectory.cpp
        src/Algorithms/reactive_planners/polynomial_end_effector_trajectory.cpp
        src/Algorithms/reactive_planners/stepper_head.cpp
)
ament_target_dependencies(test_controllers_cpp rclcpp std_msgs sensor_msgs gazebo_msgs std_srvs eiquadprog)
target_link_libraries(test_controllers_cpp pinocchio::pinocchio /opt/openrobots/lib/libeiquadprog.so)
target_link_libraries(test_controllers_cpp eigen-quadprog::eigen-quadprog)

install(TARGETS
        test_controllers_cpp
        DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY config
        DESTINATION share/${PROJECT_NAME})

# Python modules
ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
        ros2_control_test_nodes/test_controllers.py
        DESTINATION lib/${PROJECT_NAME}
)

# pybind for reactive planner module based on
# https://answers.ros.org/question/362178/create-a-python-module-with-pybind11-and-use-it-in-another-package/
# https://github.com/machines-in-motion/reactive_planners/blob/master/CMakeLists.txt

# Source files
set(reactive_planners_lib_src
        src/Algorithms/reactive_planners/stepper_head.cpp
        src/Algorithms/reactive_planners/dcm_vrp_planner.cpp
        src/Algorithms/reactive_planners/polynomial_end_effector_trajectory.cpp
        src/Algorithms/reactive_planners/dynamically_consistent_end_effector_trajectory.cpp
        src/Algorithms/reactive_planners/dcm_reactive_stepper.cpp
        src/Algorithms/reactive_planners/quadruped_dcm_reactive_stepper.cpp
)
# Create library target
add_library(${PROJECT_NAME}_reactive_planners_lib SHARED ${reactive_planners_lib_src})
# Includes.
target_include_directories(
        ${PROJECT_NAME}_reactive_planners_lib PUBLIC $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)
# Dependencies.
target_link_libraries(${PROJECT_NAME}_reactive_planners_lib pinocchio::pinocchio)
target_link_libraries(${PROJECT_NAME}_reactive_planners_lib Eigen3::Eigen)
target_link_libraries(${PROJECT_NAME}_reactive_planners_lib eigen-quadprog::eigen-quadprog)
# Export.
list(APPEND all_targets ${PROJECT_NAME}_reactive_planners_lib)

pybind11_add_module(reactive_planners_cpp
        src/Algorithms/reactive_planners/srcpy/reactive_planners.cpp
        src/Algorithms/reactive_planners/srcpy/stepper_head.cpp
        src/Algorithms/reactive_planners/srcpy/dcm_vrp_planner.cpp
        src/Algorithms/reactive_planners/srcpy/polynomial_end_effector_trajectory.cpp
        src/Algorithms/reactive_planners/srcpy/dcm_reactive_stepper.cpp
        src/Algorithms/reactive_planners/srcpy/quadruped_dcm_reactive_stepper.cpp
)
target_include_directories(
        reactive_planners_cpp
        PUBLIC $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)
target_link_libraries(reactive_planners_cpp PRIVATE pybind11::module)
target_link_libraries(reactive_planners_cpp PRIVATE ${PROJECT_NAME}_reactive_planners_lib)
target_link_boost_python(reactive_planners_cpp PRIVATE)
install(TARGETS
        reactive_planners_cpp
        DESTINATION
        ${CMAKE_INSTALL_PREFIX}/../../src/ros2_controllers_solo/ros2_control_test_nodes/ros2_control_test_nodes/reactive_planners
)

if (BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    # the following line skips the linter which checks for copyrights
    # uncomment the line when a copyright and license is not present in all source files
    #set(ament_cmake_copyright_FOUND TRUE)
    # the following line skips cpplint (only works in a git repo)
    # uncomment the line when this package is not in a git repo
    #set(ament_cmake_cpplint_FOUND TRUE)
    ament_lint_auto_find_test_dependencies()
endif ()

ament_package()
