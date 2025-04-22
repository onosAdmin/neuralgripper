#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json

class DictionaryPublisher(Node):
    def __init__(self):
        super().__init__('dictionary_publisher')
        self.publisher_ = self.create_publisher(String, 'dictionary_topic', 10)
        timer_period = 2.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.get_logger().info('Dictionary Publisher Node has been started')

    def timer_callback(self):
        # Create a sample dictionary
        sample_dict = {
            'name': 'ROS2 Node',
            'value': 42,
            'items': ['apple', 'banana', 'cherry'],
            'nested': {'key': 'value'}
        }
        
        # Convert dictionary to JSON string
        msg = String()
        msg.data = json.dumps(sample_dict)
        
        # Publish the message
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: {msg.data}')

def main(args=None):
    rclpy.init(args=args)
    dictionary_publisher = DictionaryPublisher()
    rclpy.spin(dictionary_publisher)
    dictionary_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()