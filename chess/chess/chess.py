#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

class ChessNode(Node):
    def __init__(self):
        super().__init__('chess_node')
        print('chess node started')


def main(args=None):
    rclpy.init(args=args)
    chess_node = ChessNode()
    rclpy.spin(chess_node)
    chess_node.destroy_node()
    rclpy.shutdown()