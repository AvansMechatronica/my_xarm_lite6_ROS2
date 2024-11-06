import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.launch_context import LaunchContext
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_setup_assistant_launch
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

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

    robot_ip = LaunchConfiguration('robot_ip', default='192.168.1.162')
    report_type = LaunchConfiguration('report_type', default='normal')
    
    prefix = LaunchConfiguration('prefix', default='')
    hw_ns = LaunchConfiguration('hw_ns', default='ufactory')
    limited = LaunchConfiguration('limited', default=False)
    effort_control = LaunchConfiguration('effort_control', default=False)
    velocity_control = LaunchConfiguration('velocity_control', default=False)
    add_gripper = LaunchConfiguration('add_gripper', default=False)
    add_vacuum_gripper = LaunchConfiguration('add_vacuum_gripper', default=False)

    add_realsense_d435i = LaunchConfiguration('add_realsense_d435i', default=False)

    add_other_geometry = LaunchConfiguration('add_other_geometry', default=False)
    geometry_type = LaunchConfiguration('geometry_type', default='box')
    geometry_mass = LaunchConfiguration('geometry_mass', default=0.1)
    geometry_height = LaunchConfiguration('geometry_height', default=0.1)
    geometry_radius = LaunchConfiguration('geometry_radius', default=0.1)
    geometry_length = LaunchConfiguration('geometry_length', default=0.1)
    geometry_width = LaunchConfiguration('geometry_width', default=0.1)
    geometry_mesh_filename = LaunchConfiguration('geometry_mesh_filename', default='')
    geometry_mesh_origin_xyz = LaunchConfiguration('geometry_mesh_origin_xyz', default='"0 0 0"')
    geometry_mesh_origin_rpy = LaunchConfiguration('geometry_mesh_origin_rpy', default='"0 0 0"')
    geometry_mesh_tcp_xyz = LaunchConfiguration('geometry_mesh_tcp_xyz', default='"0 0 0"')
    geometry_mesh_tcp_rpy = LaunchConfiguration('geometry_mesh_tcp_rpy', default='"0 0 0"')
    
    ros2_control_plugin = LaunchConfiguration('ros2_control_plugin', default='uf_robot_hardware/UFRobotSystemHardware')
    ros2_control_plugin = LaunchConfiguration('ros2_control_plugin', default='uf_robot_hardware/UFRobotFakeSystemHardware')

    xacro_file = LaunchConfiguration('xacro_file', default=PathJoinSubstitution([FindPackageShare('xarm_description'), 'urdf', 'xarm_device.urdf.xacro']))

    # robot ros2 control launch
    # xarm_controller/launch/_robot_ros2_control.launch.py
    robot_ros2_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare('xarm_controller'), 'launch', '_robot_ros2_control.launch.py'])),
        launch_arguments={
            'robot_ip': robot_ip,
#            'report_type': report_type,
#            'prefix': prefix,
#            'hw_ns': hw_ns,
#            'limited': limited,
#            'effort_control': effort_control,
#            'velocity_control': velocity_control,
#            'add_gripper': add_gripper,
#            'add_vacuum_gripper': add_vacuum_gripper,
            'dof': '6',
            'robot_type': 'lite',
#            'add_realsense_d435i': add_realsense_d435i,
#            'add_other_geometry': add_other_geometry,
#            'geometry_type': geometry_type,
#            'geometry_mass': geometry_mass,
#            'geometry_height': geometry_height,
#            'geometry_radius': geometry_radius,
#            'geometry_length': geometry_length,
#            'geometry_width': geometry_width,
#            'geometry_mesh_filename': geometry_mesh_filename,
#            'geometry_mesh_origin_xyz': geometry_mesh_origin_xyz,
#            'geometry_mesh_origin_rpy': geometry_mesh_origin_rpy,
#            'geometry_mesh_tcp_xyz': geometry_mesh_tcp_xyz,
#            'geometry_mesh_tcp_rpy': geometry_mesh_tcp_rpy,
            'ros2_control_plugin' : ros2_control_plugin,
            'xacro_file': xacro_file,
        }.items(),
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
            #lite6_planner_fake_include,
            #lite6_driver_include,
            robot_ros2_control_launch,
            #spawn_controllers_include,
            move_group_include,
            #moveit_rviz_include,
            rviz_node,
        ]
    )
