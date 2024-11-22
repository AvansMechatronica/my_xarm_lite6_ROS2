import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.launch_context import LaunchContext
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_setup_assistant_launch
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import Command

def launch_setup(context, *args, **kwargs):

    #def generate_launch_description():

    move_group_path = os.path.join(
        get_package_share_directory('my_lite6_moveit_config'),
        'launch',
        'move_group.launch.py'  # assuming converted to Python
    )
    move_group_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(move_group_path),
    )

    move_node = Node(
        package="my_lite6_demo",
        executable="move_joint",
        name="move_joint",
    )


    return[
            move_group_include,
            move_node,
        ]


def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function=launch_setup)
    ])