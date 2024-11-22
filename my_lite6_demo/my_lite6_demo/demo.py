#!/usr/bin/env python3
from threading import Thread

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node

from pymoveit2 import MoveIt2, MoveIt2State
from rclpy.parameter import Parameter
from ament_index_python.packages import get_package_share_directory
import xml.etree.ElementTree as ET


prefix = ''
joint_names = [
        prefix + "joint1",
        prefix + "joint2",
        prefix + "joint3",
        prefix + "joint4",
        prefix + "joint5",
        prefix + "joint6",
    ]

#base_link_name = "xarm_link"
base_link_name = "link_base"
end_effector_name = "link6"
group_name = "lite6"

package_name = 'my_lite6_moveit_config'
srdf_file_name = 'config/lite6_robot.srdf'

class srdfJointValues():
    def __init__(self, node, ros_package, srd_file_name, group_name):
        self.node = node
        self.groupname = group_name

        package_path = get_package_share_directory(ros_package)
        xml_file_path = f"{package_path}/{srd_file_name}"

        #self.node.get_logger().info(f"Reading XML file: {xml_file_path}")
        
        self.tree = ET.parse(xml_file_path)
        self.root = self.tree.getroot()

    def get_joint_values(self, name):
        right_group_state = None
        for group_state in self.root.findall("group_state"):
            if group_state.get("name") == name:
                right_group_state = group_state
                break

        # Extract joint values if the 'right' group state was found
        if right_group_state:
            joint_values= []
            joint_tmp = {}
            for joint in right_group_state.findall("joint"):
                joint_name = joint.get("name")
                joint_value = float(joint.get("value"))  # Convert the value to float
                joint_values.append(joint_value);
                joint_tmp[joint_name] = joint_value

            # Print the joint values
            #print("Joint values for 'right':", joint_tmp)
            return True, joint_values
        else:
            #print("Group state 'right' not found in the XML.")
            return False, []

class MovegroupHelper:
    def __init__(self, node, joint_names, base_link_name, end_effector_name, group_name):

        self.node = node

        # Create callback group that allows execution of callbacks in parallel without restrictions
        self.callback_group = ReentrantCallbackGroup()

        # Create MoveIt 2 interface
        self.moveit2 = MoveIt2(
            node=self.node,
            joint_names=joint_names,
            base_link_name=base_link_name,
            end_effector_name=end_effector_name,
            group_name=group_name,
            callback_group=self.callback_group,
        )

        self.moveit2.planner_id = (
            "RRTConnectkConfigDefault"
        )

        # Scale down velocity and acceleration of joints (percentage of maximum)
        self.moveit2.max_velocity = 0.5
        self.moveit2.max_acceleration = 0.5
        self.synchronous = True
        self.cancel_after_secs = 0.0

        self.cartesian = True
        self.cartesian_max_step = 0.0025
        self.cartesian_fraction_threshold = 0.0
        self.cartesian_jump_threshold = 0.0
        self.cartesian_avoid_collisions = False

    def joint_goal(self, joint_values):
        self.node.get_logger().info(f"Moving to {{joint_positions: {list(joint_values)}}}")
        self.moveit2.move_to_configuration(joint_values)
        if self.synchronous:
            # Note: the same functionality can be achieved by setting
            # `synchronous:=false` and `cancel_after_secs` to a negative value.
            self.moveit2.wait_until_executed()
        else:
            # Wait for the request to get accepted (i.e., for execution to start)
            print("Current State: " + str(self.moveit2.query_state()))
            rate = self.node.create_rate(10)
            while self.moveit2.query_state() != MoveIt2State.EXECUTING:
                rate.sleep()

            # Get the future
            print("Current State: " + str(self.moveit2.query_state()))
            future = self.moveit2.get_execution_future()

            # Cancel the goal
            if self.cancel_after_secs > 0.0:
                # Sleep for the specified time
                sleep_time = self.node.create_rate(self.cancel_after_secs)
                sleep_time.sleep()
                # Cancel the goal
                print("Cancelling goal")
                self.moveit2.cancel_execution()

            # Wait until the future is done
            while not future.done():
                rate.sleep()

            # Print the result
            print("Result status: " + str(future.result().status))
            print("Result error code: " + str(future.result().result.error_code))

    def pose_goal(self, position, orientation):
        self.moveit2.move_to_pose(
            position=position,
            quat_xyzw=orientation,
            cartesian=self.cartesian,
            cartesian_max_step=self.cartesian_max_step,
            cartesian_fraction_threshold=self.cartesian_fraction_threshold,
        )
        if self.synchronous:
            # Note: the same functionality can be achieved by setting
            # `synchronous:=false` and `cancel_after_secs` to a negative value.
            self.moveit2.wait_until_executed()
        else:
            # Wait for the request to get accepted (i.e., for execution to start)
            print("Current State: " + str(self.moveit2.query_state()))
            rate = self.node.create_rate(10)
            while self.moveit2.query_state() != MoveIt2State.EXECUTING:
                rate.sleep()

            # Get the future
            print("Current State: " + str(self.moveit2.query_state()))
            future = self.moveit2.get_execution_future()

            # Cancel the goal
            if self.cancel_after_secs > 0.0:
                # Sleep for the specified time
                sleep_time = self.node.create_rate(self.cancel_after_secs)
                sleep_time.sleep()
                # Cancel the goal
                print("Cancelling goal")
                self.moveit2.cancel_execution()

            # Wait until the future is done
            while not future.done():
                rate.sleep()

            # Print the result
            print("Result status: " + str(future.result().status))
            print("Result error code: " + str(future.result().result.error_code))


joint_states = ['left', 'right', 'home']

def main():
    rclpy.init()

    # Create node for this example
    node = Node("ex_joint_goal")

    lite6 = srdfJointValues(node, package_name, srdf_file_name, group_name)
    move_group_helper = MovegroupHelper(node, joint_names, base_link_name, end_effector_name, group_name)

    # Spin the node in background thread(s) and wait a bit for initialization
    executor = rclpy.executors.MultiThreadedExecutor(2)
    executor.add_node(node)
    executor_thread = Thread(target=executor.spin, daemon=True, args=())
    executor_thread.start()
    node.create_rate(1.0).sleep()


    for joint_state in joint_states:
        # Move to joint configuration
        result, joint_values = lite6.get_joint_values(joint_state)
        if result:
            move_group_helper.joint_goal(joint_values)

    move_group_helper.pose_goal([0.5, 0.1, 0.25], [1.0, 0.0, 0.0, 0.0])

    rclpy.shutdown()
    executor_thread.join()
    exit(0)

if __name__ == "__main__":
    main()
