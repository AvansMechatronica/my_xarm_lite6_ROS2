
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Start the template publisher ROS node
        Node(
            package='demo',
            executable='demo',
            name='demo_node',
            output='screen'
        ),
        

    ])
