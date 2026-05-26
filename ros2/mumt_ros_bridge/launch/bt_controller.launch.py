import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory("mumt_ros_bridge")
    default_params = os.path.join(package_share, "config", "bt_controller.yaml")
    default_xml = os.path.join(package_share, "behavior_trees", "mumt_autopilot.xml")

    params_file = LaunchConfiguration("params_file")
    xml_file = LaunchConfiguration("xml_file")

    return LaunchDescription([
        DeclareLaunchArgument(
            "params_file",
            default_value=default_params,
            description="Path to the BT controller parameter file.",
        ),
        DeclareLaunchArgument(
            "xml_file",
            default_value=default_xml,
            description="Path to the behavior tree XML file.",
        ),
        Node(
            package="mumt_ros_bridge",
            executable="bt_controller",
            name="mumt_bt_controller",
            output="screen",
            parameters=[
                params_file,
                {"xml_file": xml_file},
            ],
        ),
    ])
