#!/usr/bin/env python3
import rclpy
from ros2_control_test_nodes.node import RobotControllers


def main(args=None):
    rclpy.init(args=args)
    node = RobotControllers()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
