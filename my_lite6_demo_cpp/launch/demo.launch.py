import os
import yaml

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python.packages import get_package_share_directory

def load_file(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as file:
            return file.read()
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None

def generate_launch_description():
    #robot_description = os.path.join(get_package_share_directory("my_lite6_description"), "urdf/lite6_robot.urdf.xacro")

    if 0:
        robot_description_content = Command(
            [
                PathJoinSubstitution([FindExecutable(name="xacro")]),
                " ",
                PathJoinSubstitution([FindPackageShare("my_lite6_description"), "urdf", "lite6_robot.urdf.xacro"]), 
                " "
            ]
        )

        robot_description_semantic = {"robot_description_semantic": load_file("my_lite6_moveit_config", "srdf/lite6_robot.srdf")}

        #robot_description = {"robot_description": robot_description_content}

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare("ariac_description"), "urdf/ariac_robots", "ariac_robots.urdf.xacro"]), 
            " "
        ]
    )

    robot_description = {"robot_description": robot_description_content}



    moveit_config = (
        MoveItConfigsBuilder("my_lite6_robot", package_name="my_lite6_moveit_config")
        .robot_description(robot_description)
        .robot_description_semantic(file_path="config/lite6_robot.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )

        # Move group node
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
        #    trajectory_execution,
        #    planning_scene_monitor_parameters,
            {"use_sim_time": True},
        ],
    )    

    # RVIZ 
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("demo"), "rviz", "environment.rviz"]
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        parameters=[
            robot_description,
        #    robot_description_semantic,
        #    robot_description_kinematics,
            {"use_sim_time": True},
        ],
        arguments=['-d', rviz_config_file]
    )

    demo_node = Node(
            package='demo',
            executable='demo',
            name='demo_node',
            output='screen'
        )


    return LaunchDescription([
        move_group_node,
        demo_node,
        rviz
    ])
