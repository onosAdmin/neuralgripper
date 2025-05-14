#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
from rotating_base_planning import RotatingBaseController
from move_arm_to_radiant_position import move_arm_to_predefined_position
from gripper_controller import move_gripper_to_position

import math 
import time
import os
serial_port = '/dev/ttyACM0'



def degrees_to_radians(degrees):
    """Convert degrees to radians"""
    return degrees * (math.pi / 180.0)



class ArmControlSubscriber(Node):
    def __init__(self):
        super().__init__('arm_orchestrator')
        self.subscription = self.create_subscription(
            String,
            'yolo_data',
            self.yolo_data_rx_callback,
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

    def yolo_data_rx_callback(self, msg):
        """Handle incoming arm commands"""
        try:
            yolo_data = json.loads(msg.data)
            print(f"Received command: {yolo_data}")
            
            # Process the yolo_data (here you would send to your actual hardware)
            delta_degrees_x = yolo_data.get('x_requested_move', None)
            delta_degrees_y = yolo_data.get('y_requested_move', None)
            main_obj_centered = yolo_data.get('main_object_centered', None)
            found_objects = yolo_data.get('found_objects', None)

            
            if delta_degrees_x is not None and delta_degrees_y is not None:
                print(f"Moving to X: {delta_degrees_x}, Y: {delta_degrees_y}")
                # Here you would send the actual commands to your servos
                
            #if main_obj_centered is not None and main_obj_centered != "x":
            self.get_logger().info(f"main_obj_centered={main_obj_centered}")
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




    joint_positions_list = [
        # Joint positions expressed in degrees, then converted to radians
        
            degrees_to_radians(0.0),      # rotating_base
            degrees_to_radians(-25.0),      # joint1
            degrees_to_radians(0.0),      # joint2
            degrees_to_radians(-71.0),    # joint3
            degrees_to_radians(0.0),      # joint4
            degrees_to_radians(-81.0),    # joint5
        

    ]

    #motion_result = move_arm_to_predefined_position(position="man_scan")
    motion_result = move_arm_to_predefined_position(joint_positions_list=joint_positions_list)

    if motion_result:
        print("\nMotion TO START position completed successfully!")
    else:
        print("\nMotion TO START position failed. Please check the error messages.")

    time.sleep(1)


    motion_result = move_gripper_to_position(joint6_deg=-70.0, clamp_deg=30.0)
    clamp_deg = 158

    command = f"x,x,x,x,x,x,x,{clamp_deg};"
    command = command.encode('utf-8')
    os.system('''echo "'''+ command.decode() +'''" > '''+serial_port)

    if motion_result:
        print("\n Gripper Motion TO START position completed successfully!")
    else:
        print("\n Gripper Motion TO START position failed. Please check the error messages.")


    time.sleep(2)


    clamp_deg = 45
    command = f"x,x,x,x,x,x,x,{clamp_deg};"
    command = command.encode('utf-8')
    os.system('''echo "'''+ command.decode() +'''" > '''+serial_port)



    time.sleep(1)
    rclpy.init(args=args)
    arm_subscriber = ArmControlSubscriber()
    controller = RotatingBaseController()

    rclpy.spin(arm_subscriber)
    
    arm_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
