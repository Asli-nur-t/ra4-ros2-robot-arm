"""aslikol Gazebo simulasyonunu baslatir.

Baslatilanlar:
  1. Gazebo (aslikol_world.sdf)
  2. robot_state_publisher (URDF -> /robot_description ve TF)
  3. Robotun Gazebo'ya eklenmesi (ros_gz_sim create)
  4. ros_gz_bridge (saat, IMU, kamera, link pozlari)
  5. Kontrolculer: joint_state_broadcaster -> arm_controller (sirayla)
  6. ee_pose_publisher (uc efektorun tabana gore konumu)
  7. (istege bagli) RViz

Kullanim:
  ros2 launch aslikol_bringup sim.launch.py
  ros2 launch aslikol_bringup sim.launch.py rviz:=true
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (AppendEnvironmentVariable, DeclareLaunchArgument,
                            IncludeLaunchDescription, RegisterEventHandler)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    desc_share = get_package_share_directory('aslikol_description')
    bringup_share = get_package_share_directory('aslikol_bringup')
    ros_gz_sim_share = get_package_share_directory('ros_gz_sim')

    xacro_file = os.path.join(desc_share, 'urdf', 'aslikol.urdf.xacro')
    controllers_file = os.path.join(bringup_share, 'config', 'controllers.yaml')
    bridge_file = os.path.join(bringup_share, 'config', 'bridge.yaml')
    world_file = os.path.join(bringup_share, 'worlds', 'aslikol_world.sdf')
    rviz_file = os.path.join(desc_share, 'rviz', 'aslikol.rviz')

    robot_description = ParameterValue(
        Command(['xacro ', xacro_file, ' sim:=true', ' controllers_file:=', controllers_file]),
        value_type=str)

    use_rviz = LaunchConfiguration('rviz')

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_share, 'launch', 'gz_sim.launch.py')),
        launch_arguments={
            'gz_args': f'-r -v2 {world_file}',
            'on_exit_shutdown': 'true',
        }.items())

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}],
        output='screen')

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description', '-name', 'aslikol', '-z', '0.0'],
        output='screen')

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{'config_file': bridge_file, 'use_sim_time': True}],
        output='screen')

    joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster',
                   '--controller-manager', '/controller_manager',
                   '--controller-manager-timeout', '60',
                   '--param-file', controllers_file],
        output='screen')

    arm_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['arm_controller',
                   '--controller-manager', '/controller_manager',
                   '--controller-manager-timeout', '60',
                   '--param-file', controllers_file],
        output='screen')

    ee_pose_publisher = Node(
        package='aslikol_control',
        executable='ee_pose_publisher',
        parameters=[{'use_sim_time': True}],
        output='screen')

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_file],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(use_rviz),
        output='screen')

    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='false',
                              description='RViz penceresini de ac'),
        # package:// URI'lerinin Gazebo tarafinda cozulebilmesi icin
        AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', os.path.dirname(desc_share)),
        gazebo,
        robot_state_publisher,
        spawn_robot,
        bridge,
        ee_pose_publisher,
        rviz,
        # Kontrolculer robot Gazebo'ya eklendikten sonra sirayla yuklenir
        RegisterEventHandler(OnProcessExit(target_action=spawn_robot,
                                           on_exit=[joint_state_broadcaster])),
        RegisterEventHandler(OnProcessExit(target_action=joint_state_broadcaster,
                                           on_exit=[arm_controller])),
    ])
