import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.launch_context import LaunchContext
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_setup_assistant_launch
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    moveit_config = MoveItConfigsBuilder("lite6_robot", package_name="my_lite6_moveit_config").to_moveit_configs()

    if 0:
        lite6_planner_realmove_path = os.path.join(
            get_package_share_directory('xarm_planner'),
            'launch',
            'lite6_planner_fake.launch.py'  # assuming converted to Python
        )
        lite6_planner_realmove_include = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(lite6_planner_realmove_path),
            #launch_arguments={'robot_ip': LaunchConfiguration('robot_ip')}.items()
            launch_arguments={'robot_ip': '192.168.1.162'}.items()
        )

        lite6_planner_fake_path = os.path.join(
            get_package_share_directory('xarm_planner'),
            'launch',
            'lite6_planner_fake.launch.py'  # assuming converted to Python
        )
        lite6_planner_fake_include = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(lite6_planner_fake_path)
        )


    lite6_driver_path = os.path.join(
        get_package_share_directory('xarm_api'),
        'launch',
        'lite6_driver.launch.py'  # assuming converted to Python
    )
    lite6_driver_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(lite6_driver_path),
        #launch_arguments={'robot_ip': LaunchConfiguration('robot_ip')}.items()
        launch_arguments={'robot_ip': '192.168.1.162'}.items()
    )

    move_group_path = os.path.join(
        get_package_share_directory('my_lite6_moveit_config'),
        'launch',
        'move_group.launch.py'  # assuming converted to Python
    )
    move_group_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(move_group_path),
    )

    spawn_controllers_path = os.path.join(
        get_package_share_directory('my_lite6_moveit_config'),
        'launch',
        'spawn_controllers.launch.py'  # assuming converted to Python
    )
    spawn_controllers_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(spawn_controllers_path),
    )

    if 0:
        moveit_rviz_path = os.path.join(
            get_package_share_directory('my_lite6_moveit_config'),
            'launch',
            'moveit_rviz.launch.py'  # assuming converted to Python
        )
        moveit_rviz_include = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(moveit_rviz_path),
        )

    # RViz
    rviz_config_file = (
        get_package_share_directory("my_lite6_bringup") + "/rviz/moveit.rviz"
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
        ],
    )

    return LaunchDescription(
        [
            lite6_planner_fake_include,
            lite6_driver_include,
            spawn_controllers_include,
            move_group_include,
            #moveit_rviz_include,
            rviz_node,
        ]
    )
