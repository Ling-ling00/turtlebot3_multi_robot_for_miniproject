#!/usr/bin/env python3
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Authors: Arshad Mehmood

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.event_handlers import OnProcessExit
from launch.conditions import IfCondition


def generate_launch_description():
    ld = LaunchDescription()

    TURTLEBOT3_MODEL = "burger"

    declare_enable_drive = DeclareLaunchArgument(
        name="enable_drive", default_value="true", description="Enable robot drive node"
    )

    turtlebot3_multi_robot = get_package_share_directory("turtlebot3_multi_robot")

    world = os.path.join(
        turtlebot3_multi_robot, "worlds", "custom_world.world"
    )

    urdf_file_name = "turtlebot3_" + TURTLEBOT3_MODEL + ".urdf"
    print("urdf_file_name : {}".format(urdf_file_name))

    urdf = os.path.join(
        turtlebot3_multi_robot, "urdf", urdf_file_name
    )

    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("gazebo_ros"), "launch", "gzserver.launch.py")
        ),
        launch_arguments={"world": world}.items(),
    )

    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("gazebo_ros"), "launch", "gzclient.launch.py")
        ),
    )

    ld.add_action(declare_enable_drive)
    ld.add_action(gzserver_cmd)
    ld.add_action(gzclient_cmd)

    ROWS = 2
    COLS = 2

    x = -ROWS
    y = -COLS
    last_action = None

    # Remapping is required for state publisher otherwise /tf and /tf_static will get be published on root '/' namespace
    remappings = [("/tf", "tf"), ("/tf_static", "tf_static")]

    x = ["-2.0", "2.0", "-2.0", "2.0"]
    y = ["-2.0", "-2.0", "2.0", "2.0"]
    last_action = None
    name = ["robot1", "robot2", "robot3", "robot4"]
    for i in range(len(name)):
        # Create state publisher node for that instance
        turtlebot_state_publisher = Node(
            package="robot_state_publisher",
            namespace=name[i],
            executable="robot_state_publisher",
            output="screen",
            parameters=[{"use_sim_time": False,
                            "publish_frequency": 10.0}],
            remappings=remappings,
            arguments=[urdf],
        )

        # Create spawn call
        spawn_turtlebot3_burger = Node(
            package="gazebo_ros",
            executable="spawn_entity.py",
            arguments=[
                "-file",
                os.path.join(turtlebot3_multi_robot,'models', 'turtlebot3_' + TURTLEBOT3_MODEL, 'model.sdf'),
                "-entity",name[i],
                "-robot_namespace",name[i],
                "-x",x[i],
                "-y",y[i],
                "-z","0.01",
                "-unpause",
            ],
            output="screen",
        )
        if last_action is None:
            # Call add_action directly for the first robot to facilitate chain instantiation via RegisterEventHandler
            ld.add_action(turtlebot_state_publisher)
            ld.add_action(spawn_turtlebot3_burger)
            
        else:
            # Use RegisterEventHandler to ensure next robot creation happens only after the previous one is completed.
            # Simply calling ld.add_action for spawn_entity introduces issues due to parallel run.
            spawn_turtlebot3_event = RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=last_action,
                    on_exit=[spawn_turtlebot3_burger,
                                turtlebot_state_publisher],
                )
            )
            ld.add_action(spawn_turtlebot3_event)
        last_action = spawn_turtlebot3_burger


    # Create state publisher node for that instance
    turtlebot_state_publisher = Node(
        package="robot_state_publisher",
        namespace="robot5",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{"use_sim_time": False,
                        "publish_frequency": 10.0}],
        remappings=remappings,
        arguments=[urdf],
    )

    # Create spawn call
    spawn_turtlebot3_burger = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-file",
            os.path.join(turtlebot3_multi_robot,'models', 'turtlebot3_' + TURTLEBOT3_MODEL, 'model1.sdf'),
            "-entity","robot5",
            "-robot_namespace","robot5",
            "-x","0.0",
            "-y","0.0",
            "-z","0.01",
            "-unpause",
        ],
        output="screen",
    )
    if last_action is None:
        # Call add_action directly for the first robot to facilitate chain instantiation via RegisterEventHandler
        ld.add_action(turtlebot_state_publisher)
        ld.add_action(spawn_turtlebot3_burger)
        
    else:
        # Use RegisterEventHandler to ensure next robot creation happens only after the previous one is completed.
        # Simply calling ld.add_action for spawn_entity introduces issues due to parallel run.
        spawn_turtlebot3_event = RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=last_action,
                on_exit=[spawn_turtlebot3_burger,
                            turtlebot_state_publisher],
            )
        )
        ld.add_action(spawn_turtlebot3_event)
    last_action = spawn_turtlebot3_burger

    return ld
