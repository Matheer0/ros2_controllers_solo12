//
// Created by stanley on 6/6/22.
//
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "ros2_control_test_nodes/ControllersNode.hpp"

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ControllersNode>());
    rclcpp::shutdown();
    return 0;
}