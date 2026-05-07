#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, TimerAction, SetEnvironmentVariable


def generate_launch_description():

    gazebo_ros_pkg = get_package_share_directory("gazebo_ros")
    pkg_path = get_package_share_directory('trace_car')
    model_path = os.path.join(pkg_path, 'models', 'car.model')
    controllers_yaml = os.path.join(pkg_path, 'config', 'controllers.yaml')

    world = os.path.join(
        get_package_share_directory('trace_car'),
        'worlds',
        'trace.world'
    )

    env_cmd = SetEnvironmentVariable(
        name='GAZEBO_PLUGIN_PATH',
        value=os.path.join(
            get_package_share_directory('trace_car'),
            '..',
            'lib'
        ) + ':' + os.environ.get('GAZEBO_PLUGIN_PATH', '')
    )

    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_pkg, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world}.items()
    )

    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_pkg, 'launch', 'gzclient.launch.py')
        )
    )


    car_model_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("trace_car"),
                "launch",
                "car_model.launch.py")
        )
    )


    # start joint_state_broadcaster
    jstate_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
    )

    # start wheel velocity controller
    wheel_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['wheel_velocity_controller'],
    )

    # statr steer position controller
    steer_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['steer_position_controller'],
    )

    # wait for the robot to be spawned before starting the controllers
    jstate_spawner = TimerAction(period=1.0, actions=[jstate_broadcaster])
    wheel_spawner = TimerAction(period=2.0, actions=[wheel_controller])
    steer_spawner = TimerAction(period=2.0, actions=[steer_controller])

    # ccd node launch
    ccd_node_launch = Node(
        package='trace_car',
        executable='ccd_node',
        arguments=['--gui']
    )

    # controller node launch
    controller_node_launch = Node(
        package='trace_car',
        executable='controller_node',
        arguments=['--gui']
    )

    # gui launch
    gui_launch = Node(
        package='trace_car',
        executable='monitor_gui.py',
        arguments=['--ros2']
    )

    return LaunchDescription(
        [
            env_cmd,
            gzserver_cmd,
            gzclient_cmd,
            car_model_launch,
            jstate_spawner,
            wheel_spawner,
            steer_spawner,
            ccd_node_launch,
            controller_node_launch,
            gui_launch
        ]
    )
