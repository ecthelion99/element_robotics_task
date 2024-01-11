#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty
import threading

class CancelNode(Node):
    def __init__(self):
        super().__init__('cancel_node')
        self.cancel_thread = threading.Thread(target=self.check_for_cancel)
        self.cancel_thread.start()

    def check_for_cancel(self):
        while True:
            input("Press Enter to ESTOP...")
            self.send_empty_message()

    def send_empty_message(self):
        publisher = self.create_publisher(Empty, '/estop', 10)
        msg = Empty()
        publisher.publish(msg)
        self.get_logger().info('Empty message sent to /estop topic')

def main(args=None):
    rclpy.init(args=args)
    cancel_node = CancelNode()
    rclpy.spin(cancel_node)
    cancel_node.cancel_thread.join()
    cancel_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()