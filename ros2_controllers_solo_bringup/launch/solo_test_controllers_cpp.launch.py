from launch import LaunchDescription
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    efforts = PathJoinSubstitution(
        [
            FindPackageShare("ros2_controllers_solo_bringup"),
            "config",
            "solo_test_controllers.yaml",
        ]
    )

    return LaunchDescription(
        [
            Node(
                package="ros2_control_test_nodes",
                executable="test_controllers_cpp",
                name="test_controllers_cpp",
                parameters=[efforts],
                output={
                    "stdout": "screen",
                    "stderr": "screen",
                },
            )
        ]
    )