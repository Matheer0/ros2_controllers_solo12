//
// Created by stanley on 6/6/22.
//

#ifndef ROS2_CONTROL_TEST_NODES_CONTROLLERS_HPP
#define ROS2_CONTROL_TEST_NODES_CONTROLLERS_HPP

#include <filesystem>
#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "gazebo_msgs/msg/link_states.hpp"
#include "std_srvs/srv/trigger.hpp"
#include <Eigen/Dense>
#include <unordered_map>
#include <vector>
#include "ros2_control_test_nodes/PD_control/PD_control.hpp"
#include "ros2_control_test_nodes/mim_control/demo_com_ctrl_cpp.hpp"
#include "ros2_control_test_nodes/reactive_planners/demo_reactive_planners_solo12_step_adjustment.hpp"

class ControllersNode : public rclcpp::Node {
public:
    ControllersNode();

    enum States {NO_EFFORT, STAND, CENTROIDAL, WALK};

private:
    // subscribers, publishers, and services
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_subscriber_;
    rclcpp::Subscription<gazebo_msgs::msg::LinkStates>::SharedPtr link_state_subscriber_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_PD;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_centroidal;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_reactive_planner;

    // topic callbacks
    void update_joint_states(const sensor_msgs::msg::JointState::SharedPtr msg);
    void update_body_state(const gazebo_msgs::msg::LinkStates::SharedPtr msg);

    void timer_callback();

    // fields
    std::string robot_description;
    Eigen::VectorXd joint_config = Eigen::VectorXd::Zero(12);
    Eigen::VectorXd joint_velocity = Eigen::VectorXd::Zero(12);
    Eigen::VectorXd robot_pose = Eigen::VectorXd::Zero(7);
    Eigen::VectorXd robot_twist = Eigen::VectorXd::Zero(6);

    // TO-DO: move the field to the PD_control class
    Eigen::Matrix<double, 12, 1> desired_config;

    // PD control
    PD_control pdControl;

    // Centroidal control
    DemoComCtrl demoComCtrl;

    // Reactive planner
    float control_time;
    DemoReactivePlanner demoReactivePlanner;
};

#endif //ROS2_CONTROL_TEST_NODES_CONTROLLERS_HPP
