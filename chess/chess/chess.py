#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import subprocess
from std_msgs.msg import String

class ChessNode(Node):
    def __init__(self):
        super().__init__('chess_node')
        self.get_logger().info('Chess node started')

        # ROS 2 publisher for the best move
        self.publisher_ = self.create_publisher(
            String, 'chess_best_move', 10
        )  # Publish to the topic `chess_best_move`

        # Timer to control the loop
        self.timer = self.create_timer(2.0, self.timer_callback)

        # Start Stockfish subprocess once and reuse
        self.stockfish_process = self.start_stockfish_process()
        self.current_position = 'startpos'

    def start_stockfish_process(self):
        """Start Stockfish as a persistent subprocess."""
        process = subprocess.Popen(
            ['stockfish'], stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True
        )
        # Initialize Stockfish with UCI protocol
        process.stdin.write('uci\n')
        process.stdin.flush()

        # Wait for 'uciok'
        while True:
            output = process.stdout.readline().strip()
            if output == 'uciok':
                break

        return process

    def timer_callback(self):
        """ROS 2 timer callback to compute and publish the best move."""
        best_move = self.get_best_move(self.current_position)
        if best_move:
            start_position = best_move[:2]
            end_position = best_move[2:]
            self.get_logger().info(f'Best move: From {start_position} To {end_position}')
            self.current_position += f' {best_move}'

            msg = String();
            msg.data = f'{start_position}->{end_position}'

            # Publish the move
            self.publisher_.publish(msg)

    def get_best_move(self, current_position):
        """Get the best move from Stockfish for the given position."""
        # Set the current position in Stockfish
        self.stockfish_process.stdin.write(f'position {current_position}\n')
        self.stockfish_process.stdin.write('go movetime 1000\n')  # Search for 1 second
        self.stockfish_process.stdin.flush()

        # Read Stockfish output and find the best move
        best_move = None
        while True:
            output = self.stockfish_process.stdout.readline().strip()
            if output.startswith('bestmove'):
                best_move = output.split(' ')[1]
                break

        return best_move

    def destroy_node(self):
        """Clean up resources on shutdown."""
        # Quit Stockfish gracefully
        if self.stockfish_process:
            self.stockfish_process.stdin.write('quit\n')
            self.stockfish_process.stdin.flush()
            self.stockfish_process.terminate()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    chess_node = ChessNode()
    try:
        rclpy.spin(chess_node)
    except KeyboardInterrupt:
        pass
    finally:
        chess_node.destroy_node()
        rclpy.shutdown()
