#!/usr/bin/env python3
import sys
import math
import copy
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from moveit_msgs.msg import DisplayTrajectory
from tf_transformations import quaternion_from_euler, quaternion_multiply

import tf2_ros
from tf2_ros import TransformListener, Buffer
import tf2_geometry_msgs

class TestNode(Node):
    def __init__(self):
        super().__init__('test_node')
        
        # Initialize tf2 and MoveIt components
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # MoveIt initialization
        moveit_commander.roscpp_initialize(sys.argv)
        self.robot = RobotCommander()
        self.scene = PlanningSceneInterface()
        self.group = MoveGroupCommander('arm')
        
        # Publisher to display planned path
        self.display_trajectory_publisher = self.create_publisher(
            DisplayTrajectory, 
            '/move_group/display_planned_path', 
            10
        )
        
        self.execute_sequence()

    def execute_sequence(self):
        # Go to home position
        self.get_logger().info("== Go to home ==")
        target_values = self.group.get_named_target_values("home")
        self.group.go(target_values, wait=True)

        # Go to left position
        self.get_logger().info("== Go to left ==")
        target_values = self.group.get_named_target_values("left")
        self.group.go(target_values, wait=True)

        # Move down by 50mm
        self.get_logger().info("== Move down, 50 mm ==")
        pose = self.group.get_current_pose()
        posetarget = copy.deepcopy(pose)
        posetarget.pose.position.z -= 0.05
        self.group.set_pose_target(posetarget.pose)
        plan = self.group.plan()
        self.group.go(wait=True)

        # Transform and move to ik_testpoint
        self.get_logger().info("== Go to test IK ==")
        try:
            transform = self.tf_buffer.lookup_transform(
                'world', 'ik_testpoint', rclpy.time.Time()
            )
            destination_pose = Pose()
            destination_pose.position = transform.transform.translation
            destination_pose.orientation = transform.transform.rotation

            # Apply rotation to the pose
            q_orig = [
                transform.transform.rotation.x, 
                transform.transform.rotation.y, 
                transform.transform.rotation.z, 
                transform.transform.rotation.w
            ]
            q_rot = quaternion_from_euler(math.pi, 0, 0)  # Rotate along x-axis
            q_new = quaternion_multiply(q_rot, q_orig)
            destination_pose.orientation.x = q_new[0]
            destination_pose.orientation.y = q_new[1]
            destination_pose.orientation.z = q_new[2]
            destination_pose.orientation.w = q_new[3]

            self.group.set_pose_target(destination_pose)
            plan = self.group.plan()
            self.group.go(wait=True)

        except Exception as e:
            self.get_logger().error(f"Failed to transform: {e}")

        # Go to right position
        self.get_logger().info("== Go to right ==")
        target_values = self.group.get_named_target_values("right")
        self.group.go(target_values, wait=True)

        # Go to home position
        self.get_logger().info("== Go to home ==")
        target_values = self.group.get_named_target_values("home")
        self.group.go(target_values, wait=True)

        # Go to resting position
        self.get_logger().info("== Go to resting ==")
        target_values = self.group.get_named_target_values("resting")
        self.group.go(target_values, wait=True)

        self.get_logger().info("== Ready ==")


def main(args=None):
    rclpy.init(args=args)
    test_node = TestNode()
    rclpy.spin(test_node)
    test_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
