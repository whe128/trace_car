#!/usr/bin/env python3

import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    model_path = os.path.join(
        get_package_share_directory('trace_car'),
        'models',
        'car.model'
    )

    # Process the model file
    f = open(model_path, 'r')
    doc = xacro.parse(f)
    xacro.process_doc(doc)
    robot_description = doc.toxml()
    f.close()

    car_state_publisher_launch = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_description
            }],
        )



    gazebo_ros_spawner_launch = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', "car",
            '-x', '1.175',
            '-y', '-2.985',
            '-z', '0.007',
            '-R', '0.0',
            '-P', '0.0',
            '-Y', '3.14159'
        ],
        output='screen',
    )



    # rviz_launch
    rviz_launch = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=['-d', [os.path.join(get_package_share_directory('trace_car'), 'rviz', 'trace.rviz')]]
    )

    return LaunchDescription(
        [
            car_state_publisher_launch,
            gazebo_ros_spawner_launch,
            #rviz_launch
        ]
    )
