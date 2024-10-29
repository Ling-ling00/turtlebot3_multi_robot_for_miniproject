#!/usr/bin/env python3

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    turtlebot_package_name = "turtlebot3_multi_robot"

    turtlebot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    get_package_share_directory(turtlebot_package_name),
                    "launch",
                    "gazebo_custom.launch.py"
                )
            ]
        )
    )

    imu = Node(
        package="imu_calibration",
        executable="imu_bridge_node.py",
        name="imu_calib"
    )

    encode = Node(
        package="gazebo_encode",
        executable="contact_state.py",
        name="contact"
    )


    # Launch!
    return LaunchDescription(
        [   
            turtlebot,
            imu,
            encode
        ]
    )