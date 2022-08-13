# Copyright 2021 Clearpath Robotics, Inc.
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
# @author Roni Kreinin (rkreinin@clearpathrobotics.com)

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory

from launch import LaunchContext, LaunchDescription, SomeSubstitutionsType, Substitution
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, , TimerAction
from launch.conditions import IfCondition, LaunchConfigurationEquals
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node


class OffsetParser(Substitution):
    def __init__(
            self,
            number: SomeSubstitutionsType,
            offset: float,
    ) -> None:
        self.__number = number
        self.__offset = offset

    def perform(
            self,
            context: LaunchContext = None,
    ) -> str:
        number = float(self.__number.perform(context))
        return f'{number + self.__offset}'


ARGUMENTS = [
    DeclareLaunchArgument('rviz', default_value='false',
                          choices=['true', 'false'],
                          description='Start rviz.'),
    DeclareLaunchArgument('use_sim_time', default_value='true',
                          choices=['true', 'false'],
                          description='use_sim_time'),
    DeclareLaunchArgument('world', default_value='empty_world',
                          description='Ignition World'),
    DeclareLaunchArgument('robot_name', default_value='cratebot',
                          description='Robot name'),
    DeclareLaunchArgument('x', default_value='0',
                          description='The x-coordinate for the robot'),
    DeclareLaunchArgument('y', default_value='0',
                          description='The y-coordinate for the robot'),
    DeclareLaunchArgument('z', default_value='0',
                          description='The x-coordinate for the robot'),
    DeclareLaunchArgument('yaw', default_value='0',
                          description='The rotation for the robot')
]


def generate_launch_description():

    # Directories
    pkg_ignition_bringup = get_package_share_directory(
        'cratebot_ignition')
    pkg_robot_description = get_package_share_directory(
        'cratebot_description')
    pkg_robot_viz = get_package_share_directory(
        'cratebot_viz')

    pkg_ros_ign_gazebo = get_package_share_directory(
        'ros_ign_gazebo')

    # Set ignition resource path
    ign_resource_path = SetEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH',
        value=[
            os.path.join(pkg_ignition_bringup, 'worlds'), ':' +
            str(Path(pkg_robot_description).parent.resolve())])

    # Paths
    ign_gazebo_launch = PathJoinSubstitution(
        [pkg_ros_ign_gazebo, 'launch', 'ign_gazebo.launch.py'])

    ign_bridge_launch = PathJoinSubstitution(
        [pkg_ignition_bringup, 'launch', 'ignition_bridge.launch.py'])
    rviz_launch = PathJoinSubstitution(
        [pkg_robot_viz, 'launch', 'view_robot.launch.py'])
    robot_description_launch = PathJoinSubstitution(
        [pkg_robot_description, 'launch', 'robot_description.launch.py'])

    # Launch configurations
    x, y, z = LaunchConfiguration('x'), LaunchConfiguration('y'), LaunchConfiguration('z')
    yaw = LaunchConfiguration('yaw')
    # robot_node_yaml_file = LaunchConfiguration('param_file')

    # Ignition gazebo
    ignition_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([ign_gazebo_launch]),
        launch_arguments=[
            ('ign_args', [
                LaunchConfiguration('world'), '.sdf',
                ' -v 4'])
        ]
    )

    # Robot description
    robot_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([robot_description_launch]),
        launch_arguments=[('use_sim_time', LaunchConfiguration('use_sim_time'))]
    )

    # Spawn Cratebot

    # Delay launch of robot description to allow Ignition to load first.
    # Prevents visual bugs in the model.
    spawn_robot = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='ros_ign_gazebo',
                executable='create',
                arguments=[
                    '-name', LaunchConfiguration('robot_name'),
                    '-x', x,
                    '-y', y,
                    '-z', z,
                    '-Y', yaw,
                    '-topic', '/robot_description'],
                output='screen')
            ]
    )

    # ROS Ign bridge
    ros_ign_bridge = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([ign_bridge_launch])
    )

    # Rviz2
    rviz2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([rviz_launch]),
        condition=IfCondition(LaunchConfiguration('rviz')),
        launch_arguments=[('use_sim_time', LaunchConfiguration('use_sim_time'))]
    )

    # Define LaunchDescription variable
    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(ign_resource_path)
    ld.add_action(ignition_gazebo)
    ld.add_action(ros_ign_bridge)
    ld.add_action(rviz2)
    ld.add_action(robot_description)
    ld.add_action(spawn_robot)
    return ld