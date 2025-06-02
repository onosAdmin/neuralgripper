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
import sys
import socket
import threading
#import queue

serial_port = '/dev/ttyACM0'

# --- Socket Server Configuration ---
HOST = '0.0.0.0'
PORT = 65432

# --- End Socket Server Configuration ---



MOVEIT_SERVER_HOST = "127.0.0.1"
MOVEIT_SERVER_PORT = 8080


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


        # --- MoveIt Socket Client ---
        self.moveit_socket = None
        # --- End MoveIt Socket Client ---


        # --- Message Queue and OS System Status ---
        #self.message_queue = queue.Queue(maxsize=1) # Queue of size 1
        self.arm_moving_event = threading.Event() # Event to signal os.system() execution
        # self.processing_thread = None # To hold the reference to the processing thread
        # self.processing_thread_active = threading.Event() # Event to signal if processing thread is active
        # --- End Message Queue and OS System Status ---

        # Latest message storage
        self.latest_msg_data = None
        self.latest_msg_lock = threading.Lock()


    def connect_to_moveit_server(self):
        """Establish connection to the MoveIt socket server"""
        try:
            if self.moveit_socket:
                self.moveit_socket.close()
            
            self.moveit_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.moveit_socket.connect((MOVEIT_SERVER_HOST, MOVEIT_SERVER_PORT))
            self.get_logger().info(f"Connected to MoveIt server at {MOVEIT_SERVER_HOST}:{MOVEIT_SERVER_PORT}")
            return True
        except Exception as e:
            self.get_logger().error(f"Failed to connect to MoveIt server: {e}")
            return False


    def send_moveit_command(self, command):
        """Send a command to the c++ socket server and get response"""
        try:
            if not self.moveit_socket:
                if not self.connect_to_moveit_server():
                    return None

            # Send command
            self.moveit_socket.sendall((command + "\n").encode('utf-8'))
            
            # Receive response
            response = self.moveit_socket.recv(1024).decode('utf-8').strip()
            return response
        except Exception as e:
            self.get_logger().error(f"Error communicating with MoveIt server: {e}")
            # Attempt to reconnect on next command
            self.moveit_socket = None
            return None

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
                    tmp_message = ""
                    for msg_str in messages:
                        if msg_str:
                            tmp_message = msg_str
                            # Store the latest message
                            # with self.latest_msg_lock:
                            #     self.latest_msg_data = msg_str

                            # Clear the queue and add the new message
                            # while not self.message_queue.empty():
                            #     try:
                            #         self.message_queue.get_nowait()
                            #     except queue.Empty:
                            #         pass

                            #self.message_queue.put(msg_str)
                            # self.get_logger().info(f"Received message from {addr}")

                            # If a processing thread is not already running, start one
                            # if not self.processing_thread_active.is_set():
                            #     self.processing_thread_active.set()
                            #     self.processing_thread = threading.Thread(target=self.process_messages_from_queue)
                            #     self.processing_thread.daemon = True
                            #     self.processing_thread.start()



                    with self.latest_msg_lock: # only get the latest message
                        self.latest_msg_data = tmp_message


                    # Send response based on arm_moving_event
                    if self.arm_moving_event.is_set():
                        conn.sendall("arm moving\n".encode('utf-8'))
                    else:
                        conn.sendall("rxok\n".encode('utf-8'))
                        self.get_logger().info(f"Received message from {addr}")


                except json.JSONDecodeError as e:
                    self.get_logger().error(f"JSON decoding error from {addr}: {e} for data: {data}")
                except socket.error as e:
                    self.get_logger().error(f"Socket receive error from {addr}: {e}")
                    break
                except Exception as e:
                    self.get_logger().error(f"Unexpected error in handle_client for {addr}: {e} on line {sys.exc_info()[2].tb_lineno}")
                    break


    def constrain(self, value, min_val, max_val):
        """Constrain a value between min and max"""
        return max(min_val, min(max_val, value))

    def map_value(self, value, in_min, in_max, out_min, out_max):
        """Map and constrain a value from one range to another"""
        value = self.constrain(value, in_min, in_max)
        return int((value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min)

    # def yolo_data_rx_callback(self, msg_data):
    #     """Handle incoming yolo_data"""
    #     try:
    #         yolo_data = json.loads(msg_data)
    #         print(f"Received command: {yolo_data}")

    #         delta_x = yolo_data.get('x_requested_move', None)
    #         delta_y = yolo_data.get('y_requested_move', None)
    #         found_objects = yolo_data.get('found_objects', None)
    #         main_obj_centered = yolo_data.get('main_object_centered', None)
    #         main_target_object = yolo_data.get('main_target_object', None)

    #         if isinstance(found_objects, str):
    #             try:
    #                 found_objects = eval(found_objects)
    #             except (SyntaxError, NameError):
    #                 self.get_logger().warning(f"Could not parse found_objects string: {found_objects}")
    #                 found_objects = []

    #         if len(found_objects) == 0:
    #             print("No objects found")
    #             return

    #         if delta_x is not None and delta_y is not None:
    #             print(f"Moving to X: {delta_x}, Y: {delta_y}")

    #         self.get_logger().info(f"main_obj_centered={main_obj_centered}")

    #         if delta_x is not None:
    #             if delta_x > 3:
    #                 delta_x = 3
    #             if delta_x < -3:
    #                 delta_x = -3

    #         if delta_y is not None:
    #             if delta_y > 10:
    #                 delta_y = 10
    #             if delta_y < -10:
    #                 delta_y = -10

    #     except json.JSONDecodeError as e:
    #         self.get_logger().error(f"JSON decoding error: {e} for message: {msg_data}")
    #     except Exception as e:
    #         self.get_logger().error(f"Error processing yolo_data: {e}")

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



    def close_gripper(self):
        clamp_deg = 89
        motion_result = move_gripper_to_position(joint6_deg=-80.0, clamp_deg=clamp_deg,init=False)

        if motion_result:
            print("\n Gripper Motion completed successfully!")

            command = f"x,x,x,x,x,x,x,{clamp_deg};"
            command = command.encode('utf-8')
            os.system('''echo "'''+ command.decode() +'''" > '''+serial_port)


        else:
            print("\n Gripper Motion failed. Please check the error messages.")




    def open_gripper(self):
        clamp_deg = 20
        motion_result = move_gripper_to_position(joint6_deg=-80.0, clamp_deg=clamp_deg,init=False)

        if motion_result:
            print("\n Gripper Motion completed successfully!")

            command = f"x,x,x,x,x,x,x,{clamp_deg};"
            command = command.encode('utf-8')
            os.system('''echo "'''+ command.decode() +'''" > '''+serial_port)


        else:
            print("\n Gripper Motion failed. Please check the error messages.")



    def get_yolo_data(self):
        """Return the latest message data and clear the queue"""

        with self.latest_msg_lock:
            self.latest_msg_data = None

        while rclpy.ok():
            if self.latest_msg_data is not None: #wait for the latest message
                data = self.latest_msg_data
                break
            time.sleep(0.1)

        return data




    def move_to_joint_positions(self, joint_positions):
        """Move the arm to specified joint positions and clear all messages"""


        # Execute movement commands
        response = None
        if joint_positions is not None and len(joint_positions) > 0:

            self.get_logger().info(f"Moving  to joint position: {joint_positions}")
            self.arm_moving_event.set()
            try:

                command = f"movetofix,{joint_positions[0]},{joint_positions[1]},{joint_positions[2]},{joint_positions[3]},{joint_positions[4]},{joint_positions[5]}"    #'movetofix,270,90,95,180,15,22' - Move all joints to specific degrees");
                self.get_logger().info(f"MoveIt command: {command}")
                response = self.send_moveit_command(command)
                if response:
                    self.get_logger().info(f"MoveIt server response: {response}")
                else:
                    self.get_logger().error("Failed to get response from MoveIt server")
            finally:
                self.arm_moving_event.clear()
        else:
            self.get_logger().error("No joint positions provided")

        return response


    def set_speed_and_accelleration(self, speed,accelleration):


        # Execute movement commands
        response = None

        self.get_logger().info(f"speed :{speed} set_speed_and_accelleration: {accelleration} ")
        self.arm_moving_event.set()
        try:

            command = f"scaling,{speed},{accelleration}"   
            self.get_logger().info(f"client command: {command}")
            response = self.send_moveit_command(command)
            if response:
                self.get_logger().info(f"server socket response: {response}")
            else:
                self.get_logger().error("Failed to get response from socket server")
        finally:
            self.arm_moving_event.clear()


        return response






    def move_arm(self, x, y, z):
        """Move the arm to specified x,y coordinates and clear all messages"""
        try:
            # Clear all pending messages
            with self.latest_msg_lock:
                self.latest_msg_data = None


            # Execute movement commands

            if x is not None and x != 0:
                # self.get_logger().info(f"Moving base to X: {x}")
                # self.arm_moving_event.set()
                # self.move_base(x)
                # self.arm_moving_event.clear()

                self.get_logger().info(f"Moving rotating base: {x}")
                self.arm_moving_event.set()
                try:
                    #if abs(x) > 4 :
                    x = x/1.5
                    if x > 0:
                        command = f"rr,{x}"
                    else:
                        x = abs(x)
                        command = f"rl,{x}"
                    response = self.send_moveit_command(command)
                    if response:
                        self.get_logger().info(f"MoveIt server response: {response}")
                    else:
                        self.get_logger().error("Failed to get response from MoveIt server")
                finally:
                    self.arm_moving_event.clear()







            elif y is not None and y != 0:
                # self.get_logger().info(f"Moving to Y: {y}")
                # self.arm_moving_event.set()
                # try:
                #     os.system("ros2 run axis_mover01 axis_mover0.1 y "+str(y))
                # finally:
                #     self.arm_moving_event.clear()

                self.get_logger().info(f"Moving to Y: {y}")
                self.arm_moving_event.set()
                try:
                    #if abs(y) > 4 :
                    y = y/1.5
                    command = f"y,{y}"
                    response = self.send_moveit_command(command)
                    if response:
                        self.get_logger().info(f"MoveIt server response: {response}")
                    else:
                        self.get_logger().error("Failed to get response from MoveIt server")
                finally:
                    self.arm_moving_event.clear()




            elif z is not None and z != 0:

                self.get_logger().info(f"Moving to Z: {z}")
                self.arm_moving_event.set()
                try:
                    #if abs(y) > 4 :
                    z = z/1.5
                    command = f"z,{z}"
                    response = self.send_moveit_command(command)
                    if response:
                        self.get_logger().info(f"MoveIt server response: {response}")
                    else:
                        self.get_logger().error("Failed to get response from MoveIt server")
                finally:
                    self.arm_moving_event.clear()


        except Exception as e:
            self.get_logger().error(f"Error in move_arm: {e}")

    def destroy_node(self):
        self.get_logger().info("Shutting down socket server.")
        self.server_socket.close()
        # if self.processing_thread and self.processing_thread.is_alive():
        #     self.get_logger().info("Waiting for processing thread to finish...")
        super().destroy_node()

def main(args=None):
    # Initial arm setup (same as original)
    joint_positions_list_start = [
        90.0,
        90.0,
        90.0,
        90.0,
        90.0,
        90.0,
    ]




    joint_positions_list_min = [
        0.0,
        5.0,
        0.0,
        -84,
        0.0,
        -90.0,
    ]


    joint_positions_list_mid = [
        0.0,
        -12.0,
        0.0,
        -67.0,
        0.0,
        -85.0,
    ]



    joint_positions_deposit_box = [
        60.0,
        -12.0,
        0.0,
        -67.0,
        0.0,
        -85.0,
    ]






    # Initial arm setup (same as original)
    joint_positions_list_max = [
        0.0,
        -30,
        0.0,
        -50.0,
        0.0,
        -88.0,
    ]



    rclpy.init(args=args)
    ros2_arm_orchestrator = ros2ArmOrchestrator()


    joint_positions_list = joint_positions_list_min
    #motion_result = move_arm_to_predefined_position(joint_positions_list=joint_positions_list)
    motion_result = ros2_arm_orchestrator.move_to_joint_positions(joint_positions_list)
    if motion_result:
        print("\nMotion TO START position completed successfully!")
    else:
        print("\nMotion TO START position failed. Please check the error messages.")



    # joint_positions_list = joint_positions_list_max

    # #motion_result = move_arm_to_predefined_position(joint_positions_list=joint_positions_list)
    # motion_result = ros2_arm_orchestrator.move_to_joint_positions(joint_positions_list)
    # if motion_result:
    #     print("\nMotion TO START position completed successfully!")
    # else:
    #     print("\nMotion TO START position failed. Please check the error messages.")





    joint_positions_list = joint_positions_list_mid
    
    ros2_arm_orchestrator.set_speed_and_accelleration(1,1)

    #motion_result = move_arm_to_predefined_position(joint_positions_list=joint_positions_list)
    motion_result = ros2_arm_orchestrator.move_to_joint_positions(joint_positions_list)
    if motion_result:
        print("\nMotion TO START position completed successfully!")
    else:
        print("\nMotion TO START position failed. Please check the error messages.")



    
    ros2_arm_orchestrator.close_gripper()

    # clamp_deg = 45
    # command = f"x,x,x,x,x,x,x,{clamp_deg};"
    # command = command.encode('utf-8')
    # os.system('''echo "'''+ command.decode() +'''" > '''+serial_port)
    time.sleep(1)

    ros2_arm_orchestrator.open_gripper()

    object_centered_count = 0
    #ros2_arm_orchestrator.move_arm(10,0,0)

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
                    delta_x =  data.get('x_requested_move', None)
                    delta_y = data.get('y_requested_move', None)
                    found_objects = data.get('found_objects', None)
                    main_obj_centered = data.get('main_object_centered', None)
                    main_target_object = data.get('main_target_object', None)

                    if len(found_objects) == 0:
                        print("No objects found,put a scan function here")
                        continue

                    if main_obj_centered:
                        object_centered_count = object_centered_count + 1
                        if object_centered_count > 5:
                            object_centered_count = 0
                            print("Object centered and ready for pick up ,put a centering function here")
                            ros2_arm_orchestrator.move_arm(0,0,-45)
                            ros2_arm_orchestrator.move_arm(0,0,-35)
                            ros2_arm_orchestrator.move_arm(0,0,-25)
                            ros2_arm_orchestrator.move_arm(0,0,-10)
                            ros2_arm_orchestrator.move_arm(0,0,-10)
                            ros2_arm_orchestrator.move_arm(0,0,-10)

                            ros2_arm_orchestrator.close_gripper()
                            ros2_arm_orchestrator.move_arm(0,0,+65)
                            ros2_arm_orchestrator.move_arm(0,0,+35)

                            joint_positions_list = joint_positions_deposit_box
                            #motion_result = move_arm_to_predefined_position(joint_positions_list=joint_positions_list)
                            motion_result = ros2_arm_orchestrator.move_to_joint_positions(joint_positions_list)
                            if motion_result:
                                print("\nMotion TO DEPOSIT position completed successfully!")
                            else:
                                print("\nMotion TO DEPOSIT position failed. Please check the error messages.")


                            ros2_arm_orchestrator.open_gripper()

                            joint_positions_list = joint_positions_list_mid  
                            #motion_result = move_arm_to_predefined_position(joint_positions_list=joint_positions_list)
                            motion_result = ros2_arm_orchestrator.move_to_joint_positions(joint_positions_list)
                            if motion_result:
                                print("\nMotion TO MID position completed successfully!")
                            else:
                                print("\nMotion TO MID position failed. Please check the error messages.")
                        
                        
                        else:
                            print(f"Object centered  for {object_centered_count} ")
                        
                        
                        continue
                    else:
                        # Move arm if needed
                        if delta_x is not None or delta_y is not None:
                            ros2_arm_orchestrator.move_arm(delta_x, delta_y,0)

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
