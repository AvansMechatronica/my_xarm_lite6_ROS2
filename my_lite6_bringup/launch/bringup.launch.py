from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument('robot_ip', default_value='192.168.1.162'),
        DeclareLaunchArgument('report_type', default_value='normal'),
        DeclareLaunchArgument('uf_hw_ns', default_value='ufactory'),
        DeclareLaunchArgument('velocity_control', default_value='false'),
        DeclareLaunchArgument('enforce_limits', default_value='true'),
        
        # Include xarm_bringup launch
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                FindPackageShare('xarm_bringup'), '/launch/lite6_server.launch.py'
            ]),
            launch_arguments={
                'robot_ip': LaunchConfiguration('robot_ip'),
                'report_type': LaunchConfiguration('report_type'),
                'use_moveit': TextSubstitution(text='true'),
                'ns': LaunchConfiguration('uf_hw_ns'),
                'velocity_control': LaunchConfiguration('velocity_control'),
                'enforce_limits': LaunchConfiguration('enforce_limits')
            }.items()
        ),

    ])
