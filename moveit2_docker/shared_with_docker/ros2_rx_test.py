#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json

class DictionarySubscriber(Node):
    def __init__(self):
        super().__init__('dictionary_subscriber')
        self.subscription = self.create_subscription(
            String,
            'dictionary_topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.get_logger().info('Dictionary Subscriber Node has been started')

    def listener_callback(self, msg):
        # Convert JSON string back to dictionary
        received_dict = json.loads(msg.data)
        self.get_logger().info(f'I received: {received_dict}')
        
        # You can now access dictionary elements
        self.get_logger().info(f"Name: {received_dict['name']}")
        self.get_logger().info(f"Value: {received_dict['value']}")
        self.get_logger().info(f"Items: {received_dict['items']}")
        self.get_logger().info(f"Nested value: {received_dict['nested']['key']}")

def main(args=None):
    rclpy.init(args=args)
    dictionary_subscriber = DictionarySubscriber()
    rclpy.spin(dictionary_subscriber)
    dictionary_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()