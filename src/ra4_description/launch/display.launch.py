"""RA4 modelini RViz'de goruntuler (Gazebo olmadan).

Kullanim:
  ros2 launch ra4_description display.launch.py            # kaydiricili eklem paneli ile
  ros2 launch ra4_description display.launch.py gui:=false # panelsiz
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare('ra4_description')
    xacro_file = PathJoinSubstitution([pkg_share, 'urdf', 'ra4.urdf.xacro'])
    rviz_config = PathJoinSubstitution([pkg_share, 'rviz', 'ra4.rviz'])

    robot_description = ParameterValue(
        Command(['xacro ', xacro_file, ' sim:=false']), value_type=str)

    gui = LaunchConfiguration('gui')

    return LaunchDescription([
        DeclareLaunchArgument(
            'gui', default_value='true',
            description='Eklemleri kaydiricilarla hareket ettirmek icin joint_state_publisher_gui ac'),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
            output='screen'),

        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            condition=IfCondition(gui)),

        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            condition=UnlessCondition(gui)),

        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_config],
            output='screen'),
    ])
