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
import socket
import threading
import queue

serial_port = '/dev/ttyACM0'

# --- Socket Server Configuration ---
HOST = '0.0.0.0'
PORT = 65432
# --- End Socket Server Configuration ---

def degrees_to_radians(degrees):
    """Convert degrees to radians"""
    return degrees * (math.pi / 180.0)

class ros2ArmOrchestrator(Node):
    def __init__(self):
        super().__init__('ros2_arm_orchestrator')
        self.get_logger().info('ROS2 Arm Orchestrator Node has started')

        self.controller = RotatingBaseController()

        # Servo parameters
        self.list_of_servo_to_invert = ["joint0","joint11"]
        self.list_of_servo_with_270_degree = ["joint3"]

        # --- Socket Server Initialization ---
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server_socket.bind((HOST, PORT))
        self.server_socket.listen(1)
        self.get_logger().info(f"Socket server listening on {HOST}:{PORT}")
        self.client_thread = threading.Thread(target=self.accept_connections)
        self.client_thread.daemon = True
        self.client_thread.start()
        # --- End Socket Server Initialization ---

        # --- Message Queue and OS System Status ---
        self.message_queue = queue.Queue(maxsize=1) # Queue of size 1
        self.arm_moving_event = threading.Event() # Event to signal os.system() execution
        self.processing_thread = None # To hold the reference to the processing thread
        self.processing_thread_active = threading.Event() # Event to signal if processing thread is active
        # --- End Message Queue and OS System Status ---

        # Latest message storage
        self.latest_msg_data = None
        self.latest_msg_lock = threading.Lock()

    def accept_connections(self):
        while rclpy.ok():
            try:
                conn, addr = self.server_socket.accept()
                self.get_logger().info(f"Accepted connection from {addr}")
                client_handler = threading.Thread(target=self.handle_client, args=(conn, addr))
                client_handler.daemon = True
                client_handler.start()
            except socket.error as e:
                self.get_logger().error(f"Socket accept error: {e}")
                break

    def handle_client(self, conn, addr):
        with conn:
            while rclpy.ok():
                try:
                    data = conn.recv(4096).decode('utf-8')
                    if not data:
                        self.get_logger().info(f"Client {addr} disconnected.")
                        break

                    messages = data.strip().split('\n')
                    for msg_str in messages:
                        if msg_str:
                            # Store the latest message
                            with self.latest_msg_lock:
                                self.latest_msg_data = msg_str

                            # Clear the queue and add the new message
                            while not self.message_queue.empty():
                                try:
                                    self.message_queue.get_nowait()
                                except queue.Empty:
                                    pass

                            self.message_queue.put(msg_str)
                            self.get_logger().info(f"Queued message from {addr}")

                            # If a processing thread is not already running, start one
                            if not self.processing_thread_active.is_set():
                                self.processing_thread_active.set()
                                self.processing_thread = threading.Thread(target=self.process_messages_from_queue)
                                self.processing_thread.daemon = True
                                self.processing_thread.start()

                            # Send response based on arm_moving_event
                            if self.arm_moving_event.is_set():
                                conn.sendall("arm moving\n".encode('utf-8'))
                            else:
                                conn.sendall("rxok\n".encode('utf-8'))

                except json.JSONDecodeError as e:
                    self.get_logger().error(f"JSON decoding error from {addr}: {e} for data: {data}")
                except socket.error as e:
                    self.get_logger().error(f"Socket receive error from {addr}: {e}")
                    break
                except Exception as e:
                    self.get_logger().error(f"Unexpected error in handle_client for {addr}: {e}")
                    break

    def process_messages_from_queue(self):
        """Worker thread function to process messages from the queue."""
        while rclpy.ok():
            try:
                msg_data = self.message_queue.get(block=True)
                self.get_logger().info("Processing message from queue...")
                self.yolo_data_rx_callback(msg_data)
                self.message_queue.task_done()

                if self.message_queue.empty():
                    self.processing_thread_active.clear()
                    break

            except queue.Empty:
                self.get_logger().warning("Queue unexpectedly empty in processing thread.")
                self.processing_thread_active.clear()
                break
            except Exception as e:
                self.get_logger().error(f"Error in processing thread: {e}")
                self.message_queue.task_done()
                self.processing_thread_active.clear()
                break

    def constrain(self, value, min_val, max_val):
        """Constrain a value between min and max"""
        return max(min_val, min(max_val, value))

    def map_value(self, value, in_min, in_max, out_min, out_max):
        """Map and constrain a value from one range to another"""
        value = self.constrain(value, in_min, in_max)
        return int((value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min)

    def yolo_data_rx_callback(self, msg_data):
        """Handle incoming yolo_data"""
        try:
            yolo_data = json.loads(msg_data)
            print(f"Received command: {yolo_data}")

            delta_x = yolo_data.get('x_requested_move', None)
            delta_y = yolo_data.get('y_requested_move', None)
            found_objects = yolo_data.get('found_objects', None)
            main_obj_centered = yolo_data.get('main_object_centered', None)
            main_target_object = yolo_data.get('main_target_object', None)

            if isinstance(found_objects, str):
                try:
                    found_objects = eval(found_objects)
                except (SyntaxError, NameError):
                    self.get_logger().warning(f"Could not parse found_objects string: {found_objects}")
                    found_objects = []

            if len(found_objects) == 0:
                print("No objects found")
                return

            if delta_x is not None and delta_y is not None:
                print(f"Moving to X: {delta_x}, Y: {delta_y}")

            self.get_logger().info(f"main_obj_centered={main_obj_centered}")

            if delta_x is not None:
                if delta_x > 3:
                    delta_x = 3
                if delta_x < -3:
                    delta_x = -3

            if delta_y is not None:
                if delta_y > 10:
                    delta_y = 10
                if delta_y < -10:
                    delta_y = -10

        except json.JSONDecodeError as e:
            self.get_logger().error(f"JSON decoding error: {e} for message: {msg_data}")
        except Exception as e:
            self.get_logger().error(f"Error processing yolo_data: {e}")

    def move_base(self, delta_degrees):
        try:
            joint_positions = self.controller.get_current_joint_positions()
            current_angle = joint_positions['rotating_base']

            if current_angle is None:
                print("Error: Could not determine current rotating_base angle. Exiting.")
                return

            current_degrees = math.degrees(current_angle)
            target_degrees = current_degrees + delta_degrees

            target_degrees = max(-90, min(90, target_degrees))
            target_angle = math.radians(target_degrees)

            self.controller.send_goal_rotating_base(target_angle)
            success = self.controller.wait_for_result()

            if success:
                print(f"Successfully moved to {target_degrees:.2f} degrees")
            else:
                print("Failed to move to target position")
        except Exception as e:
            print(f"\nError in move_base(): {e}")

    def get_yolo_data(self):
        """Return the latest message data and clear the queue"""
        with self.latest_msg_lock:
            data = self.latest_msg_data
            self.latest_msg_data = None
            # Clear the queue
            while not self.message_queue.empty():
                try:
                    self.message_queue.get_nowait()
                except queue.Empty:
                    pass
            return data

    def move_arm(self, x, y):
        """Move the arm to specified x,y coordinates and clear all messages"""
        try:
            # Clear all pending messages
            with self.latest_msg_lock:
                self.latest_msg_data = None
            while not self.message_queue.empty():
                try:
                    self.message_queue.get_nowait()
                except queue.Empty:
                    pass

            # Execute movement commands

            if x is not None and x != 0:
                self.get_logger().info(f"Moving base to X: {x}")
                self.arm_moving_event.set()
                self.move_base(x)
                self.arm_moving_event.clear()



            elif y is not None and y != 0:
                self.get_logger().info(f"Moving to Y: {y}")
                self.arm_moving_event.set()
                try:
                    os.system("ros2 run axis_mover01 axis_mover0.1 y "+str(y))
                finally:
                    self.arm_moving_event.clear()



        except Exception as e:
            self.get_logger().error(f"Error in move_arm: {e}")

    def destroy_node(self):
        self.get_logger().info("Shutting down socket server.")
        self.server_socket.close()
        if self.processing_thread and self.processing_thread.is_alive():
            self.get_logger().info("Waiting for processing thread to finish...")
        super().destroy_node()

def main(args=None):
    # Initial arm setup (same as original)
    joint_positions_list = [
        degrees_to_radians(0.0),
        degrees_to_radians(4.0),
        degrees_to_radians(0.0),
        degrees_to_radians(-90.0),
        degrees_to_radians(0.0),
        degrees_to_radians(-76.0),
    ]

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
    ros2_arm_orchestrator = ros2ArmOrchestrator()

    # Main infinite loop
    try:
        while rclpy.ok():
            # Example usage of the new functions:
            # Get latest yolo data
            yolo_data = ros2_arm_orchestrator.get_yolo_data()
            if yolo_data:
                print(f"Latest YOLO data: {yolo_data}")
                # Parse the data and extract x,y if needed
                try:
                    data = json.loads(yolo_data)
                    x = data.get('x_requested_move', 0)
                    y = data.get('y_requested_move', 0)
                    # Move arm if needed
                    if x != 0 or y != 0:
                        ros2_arm_orchestrator.move_arm(x, y)
                except json.JSONDecodeError:
                    pass

            # Process ROS callbacks
            rclpy.spin_once(ros2_arm_orchestrator, timeout_sec=0.1)
            
    except KeyboardInterrupt:
        pass
    finally:
        ros2_arm_orchestrator.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()