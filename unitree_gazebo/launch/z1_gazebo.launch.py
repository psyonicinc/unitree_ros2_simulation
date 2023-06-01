#!/usr/bin/python3
import os

from ament_index_python.packages import get_package_share_directory, get_package_share_path, get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.conditions import UnlessCondition

import xacro


def generate_launch_description():
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    walker_sim_path = get_package_share_path('walker_sim')

    #this part helps deal with the 'model://' URI not found
    install_dir = get_package_prefix('walker_sim')

    if 'GAZEBO_MODEL_PATH' in os.environ:
        os.environ['GAZEBO_MODEL_PATH'] =  os.environ['GAZEBO_MODEL_PATH'] + ':' + install_dir + '/share'
    else:
        os.environ['GAZEBO_MODEL_PATH'] =  install_dir + "/share"

    if 'GAZEBO_PLUGIN_PATH' in os.environ:
        os.environ['GAZEBO_PLUGIN_PATH'] = os.environ['GAZEBO_PLUGIN_PATH'] + ':' + install_dir + '/lib'
    else:
        os.environ['GAZEBO_PLUGIN_PATH'] = install_dir + '/lib'

    default_model_path = walker_sim_path / 'robots/simple_walker.urdf'
    model_arg = DeclareLaunchArgument(name='model', default_value=str(default_model_path), description="absolute path to robot urdf file")

    robot_description = ParameterValue(Command(['xacro ', LaunchConfiguration('model')]),
                                       value_type=str)
        
    params =  {"robot_description": robot_description} #params for robot_state_publisher_node
    robot_state_publisher_node = Node(package = 'robot_state_publisher', 
                                      executable='robot_state_publisher', 
                                       parameters=[params],
                                      output='screen')
    
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        condition=UnlessCondition(LaunchConfiguration('gui'))
    )

    load_joint_state_broadcaster = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_state_broadcaster'],
        output='screen'
    )

    load_joint_effort_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'joint_effort_controller'],
        output='screen'
    )

    # Gazebo launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py'),
        )
    )    

    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'bipedal_walker',
                                   '-x', '0.0',
                                   '-y', '0.0',
                                   '-z', '0.0',
                                   '-Y', '0.00'],
                        output='screen')

    return LaunchDescription([
        DeclareLaunchArgument(name='gui', default_value='True',
                                            description='Flag to enable joint_state_publisher_gui'),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_entity,
                on_exit=[load_joint_state_broadcaster],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_broadcaster,
                on_exit=[load_joint_effort_controller],
            )
        ),
        gazebo,
        model_arg,
        joint_state_publisher_node,
        robot_state_publisher_node,
        spawn_entity
    ])