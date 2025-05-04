#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import serial
import os

serial_port = '/dev/ttyACM0'
list_of_servo_to_invert = ["joint10","joint11"]
list_of_servo_with_270_degree = ["joint3"]
#list_of_servo_to_invert = []

def map_value(value, in_min, in_max, out_min, out_max):
    """Maps a value from one range to another."""
    return int((value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min)

class JointStateListener(Node):
    def __init__(self):
        super().__init__('joint_state_listener')

        # Initialize serial communication
        self.ser = serial.Serial(serial_port, 115200, timeout=1)  # Change port as needed
        
        # Create a subscription to the joint_states topic
        self.subscription = self.create_subscription(
            JointState,
            'joint_states',  # The topic name
            self.joint_state_callback,
            10  # QoS profile
        )



    def joint_state_callback(self, msg):
        """Callback function to process joint state messages."""
        joint_list = ['x','x','x','x','x','x','x','x']
        #print(str(msg.name))
        for i in range(len(msg.name)):
            if msg.name[i] ==  "rotating_base":
                joint_list[0] = {"rotating_base":msg.position[i]}

            elif msg.name[i] ==  "joint1":
                joint_list[1] = {"joint1":msg.position[i]}

            elif msg.name[i] ==  "joint2":
                joint_list[2] = {"joint2":msg.position[i]}

            elif msg.name[i] ==  "joint3":
                joint_list[3] = {"joint3":msg.position[i]}

            elif msg.name[i] ==  "joint4":
                joint_list[4] = {"joint4":msg.position[i]}

            elif msg.name[i] ==  "joint5":
                joint_list[5] = {"joint5":msg.position[i]}

            elif msg.name[i] ==  "joint6":
                joint_list[6] = {"joint6":msg.position[i]}
                print(f"joint6:{msg.position[i]}")

            elif msg.name[i] ==  "clamp_moving_joint":
                joint_list[7] = {"clamp_moving_joint":msg.position[i]}
                print(f"clamp_moving_joint:{msg.position[i]}")

                
        command = ""
        #max_value = 180

        for a in joint_list:
            if a != "x":

                if list(a.keys())[0] in list_of_servo_to_invert:  #if the key is in the list_of_servo_to_invert list
                    print(" INVERTED joint"+str(a))
                    print(list(a.values())[0])
                    value_to_send = float(list(a.values())[0])
                    #if value_to_send > max_value:
                    #    value_to_send = max_value
                    if list(a.keys())[0] in list_of_servo_with_270_degree:
                        command = command + str(map_value(value_to_send,2.35619, -2.35619, 0, 270)) + ","
                    else:
                        command = command + str(map_value(value_to_send,1.5708, -1.5708, 0, 180)) + ","
                else:
                    value_to_send = float(list(a.values())[0])
                    #if value_to_send > max_value:
                    #    value_to_send = max_value

                    if list(a.keys())[0] in list_of_servo_with_270_degree:
                        command = command + str(map_value(value_to_send,-2.35619, 2.35619, 0, 270)) + "," 
                    else:
                        command = command + str(map_value(value_to_send,-1.5708, 1.5708, 0, 180)) + "," 
            else:
                command = command + "x," 
        
        command = command[0:-1] + ";"  # drop the last , and replace it with ;
        command = command.encode('utf-8')
        #self.ser.write(command)
        #self.ser.flush()  # Ensure the command is fully sent
        self.get_logger().info(f'Sent command: {command.decode()}')
        os.system('''echo "'''+ command.decode() +'''" > '''+serial_port)
        # echo "222,127,127,127,127,x;" > /dev/ttyACM0
        #os.system('''echo "222,127,127,127,127,x;" > /dev/ttyACM0''')


def main(args=None):
    rclpy.init(args=args)
    joint_state_listener = JointStateListener()
    try:
        print("Script is waiting for a ros2 message, bring up  moiveit2")
        rclpy.spin(joint_state_listener)
    except KeyboardInterrupt:
        pass
    finally:
        joint_state_listener.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
