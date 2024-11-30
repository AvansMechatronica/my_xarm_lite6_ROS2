#!/usr/bin/env python3
from threading import Thread

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node

#from ament_index_python.packages import get_package_share_directory
import xml.etree.ElementTree as ET

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from my_moveit_python import srdfGroupStates
from my_moveit_python import MovegroupHelper

prefix = ''
joint_names = [
        prefix + "joint1",
        prefix + "joint2",
        prefix + "joint3",
        prefix + "joint4",
        prefix + "joint5",
        prefix + "joint6",
    ]
base_link_name = "link_base"
end_effector_name = "link6"
group_name = "lite6"
package_name = 'my_lite6_moveit_config'
srdf_file_name = 'config/lite6_robot.srdf'

joint_states = ['left', 'right', 'home']

def main():
    rclpy.init()
    # Create node for this example
    node = Node("demo")

    node.tf_buffer = Buffer()
    node.tf_listener = TransformListener(node.tf_buffer, node)

    lite6_groupstates = srdfGroupStates(package_name, srdf_file_name, group_name)
    move_group_helper = MovegroupHelper(node, joint_names, base_link_name, end_effector_name, group_name)

    # Spin the node in background thread(s) and wait a bit for initialization
    executor = rclpy.executors.MultiThreadedExecutor(2)
    executor.add_node(node)
    executor_thread = Thread(target=executor.spin, daemon=True, args=())
    executor_thread.start()
    node.create_rate(1.0).sleep()


    for joint_state in joint_states:
        # Move to joint configuration
        result, joint_values = lite6_groupstates.get_joint_values(joint_state)
        if result:
            print("Move to " + joint_state)
            move_group_helper.move_to_configuration(joint_values)
        else:
            print( "Failed to get joint_values of " + joint_state)


    print("Move to fixed pose")
    translation = [0.5, 0.1, 0.25]
    rotation = [1.0, 0.0, 0.0, 0.0]
    move_group_helper.move_to_pose(translation, rotation)


    print("Move to published fransfer frame")
    to_frame_rel = 'xarm_link'
    from_frame_rel = 'test_transfer_frame'
    try:
        t = node.tf_buffer.lookup_transform(
            to_frame_rel,
            from_frame_rel,
            rclpy.time.Time())
        #node.get_logger().info(t)
        translation = [0.0, 0.0, 0.0]
        rotation = [0.0, 0.0, 0.0, 0.0]

        translation[0] = t.transform.translation.x
        translation[1] = t.transform.translation.y
        translation[2] = t.transform.translation.z
        rotation[0] = t.transform.rotation.w
        rotation[1] = t.transform.rotation.x
        rotation[2] = t.transform.rotation.y
        rotation[3] = t.transform.rotation.z
        move_group_helper.move_to_pose(translation, rotation)

    except TransformException as ex:
        node.get_logger().info(
            f'Could not transform {to_frame_rel} to {from_frame_rel}: {ex}')
    
    result, joint_values = lite6_groupstates.get_joint_values('home')
    if result:
        print("Move to home")
        move_group_helper.move_to_configuration(joint_values)
    else:
        print( "Failed to get joint_values of home")

    if 0:
        # testing
        result, joint_values = lite6_groupstates.get_joint_values('home')
        if result:
            move_group_helper.compute_fk(joint_values)

        translation = [0.5, 0.1, 0.25]
        rotation = [1.0, 0.0, 0.0, 0.0]
        move_group_helper.compute_ik(translation, rotation)

        move_group_helper.move_servo()

    rclpy.shutdown()
    #executor_thread.join()
    exit(0)

if __name__ == "__main__":
    main()
