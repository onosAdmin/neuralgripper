#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
from rotating_base_planning import RotatingBaseController
from move_arm_to_radiant_position import move_arm_to_predefined_position
import math 


class ArmControlSubscriber(Node):
    def __init__(self):
        super().__init__('arm_control_subscriber')
        self.subscription = self.create_subscription(
            String,
            'arm_commands',
            self.command_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.get_logger().info('Arm Control Subscriber Node has started')

        self.controller = RotatingBaseController()

        # Servo parameters
        self.list_of_servo_to_invert = ["joint0","joint11"]
        self.list_of_servo_with_270_degree = ["joint3"]

    def constrain(self, value, min_val, max_val):
        """Constrain a value between min and max"""
        return max(min_val, min(max_val, value))

    def map_value(self, value, in_min, in_max, out_min, out_max):
        """Map and constrain a value from one range to another"""
        value = self.constrain(value, in_min, in_max)
        return int((value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min)

    def command_callback(self, msg):
        """Handle incoming arm commands"""
        try:
            command = json.loads(msg.data)
            print(f"Received command: {command}")
            
            # Process the command (here you would send to your actual hardware)
            delta_degrees_x = command.get('x_requested_move', None)
            delta_degrees_y = command.get('y_requested_move', None)
            grabber_pos = command.get('grabber_requested_move', None)
            
            if delta_degrees_x is not None and delta_degrees_y is not None:
                print(f"Moving to X: {delta_degrees_x}, Y: {delta_degrees_y}")
                # Here you would send the actual commands to your servos
                
            if grabber_pos is not None and grabber_pos != "x":
                action = "Opening" if grabber_pos > 75 else "Closing"
                self.get_logger().info(f"{action} grabber to position: {grabber_pos}")
                # Here you would send the grabber command to your hardware

            if delta_degrees_x > 3:
                delta_degrees_x = 3

            if delta_degrees_x < -3:
                delta_degrees_x = -3

            self.move_base(delta_degrees_x)
            

        except json.JSONDecodeError as e:
            self.get_logger().error(f"JSON decoding error: {e}")


    def move_base(self,delta_degrees):
        try:
            # Get current joint positions
            joint_positions = self.controller.get_current_joint_positions()
            current_angle = joint_positions['rotating_base']
            
            if current_angle is None:
                print("Error: Could not determine current rotating_base angle. Exiting.")
                return
            
            current_degrees = math.degrees(current_angle)
            target_degrees = current_degrees + delta_degrees
            
            # Clamp within joint limits
            target_degrees = max(-90, min(90, target_degrees))
            target_angle = math.radians(target_degrees)
            
            # Send the goal
            self.controller.send_goal_rotating_base(target_angle)
            success = self.controller.wait_for_result()
            
            if success:
                print(f"Successfully moved to {target_degrees:.2f} degrees")
            else:
                print("Failed to move to target position")
        except Exception as e:
            print(f"\nError in move_base(): {e}")



def main(args=None):
    motion_result = move_arm_to_predefined_position(position="man_scan")
    if motion_result:
        print("\nMotion TO START position completed successfully!")
    else:
        print("\nMotion TO START position failed. Please check the error messages.")
    
    rclpy.init(args=args)
    arm_subscriber = ArmControlSubscriber()
    #controller = RotatingBaseController()

    rclpy.spin(arm_subscriber)
    
    arm_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
