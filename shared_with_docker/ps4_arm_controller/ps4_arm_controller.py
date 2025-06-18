from pyPS4Controller.controller import Controller
import time
import math
import json
import socket

#sudo pip install pyPS4Controller


# --- Socket Client Configuration ---
SERVER_HOST = 'moveit2'  # IP address of the YoloDataSubscriber server
SERVER_PORT = 65432        # Port the YoloDataSubscriber server is listening on
# --- End Socket Client Configuration ---




class MyController(Controller):

    def __init__(self, **kwargs):
        Controller.__init__(self, **kwargs)
        # --- Socket Client Initialization ---
        self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.log_info('Arm Control Node has been started (Pure Socket Client).') 
        self.run()
        # --- End Socket Client Initialization ---

    def on_x_press(self):
        message = {"cmd_type":"joystick","Joystick_x":1}
        self.send_message(message)

    def on_x_release(self):
        message = {"cmd_type":"joystick","Joystick_x":0}
        self.send_message(message)

    def on_triangle_press(self):
        message = {"cmd_type":"joystick","Joystick_triangle":1}
        self.send_message(message)

    def on_triangle_release(self):
        message = {"cmd_type":"joystick","Joystick_triangle":0}
        self.send_message(message)

    def on_circle_press(self):
        message = {"cmd_type":"joystick","Joystick_circle":1}
        self.send_message(message)

    def on_circle_release(self):
        message = {"cmd_type":"joystick","Joystick_circle":0}
        self.send_message(message)

    def on_square_press(self):
        message = {"cmd_type":"joystick","Joystick_square":1}
        self.send_message(message)

    def on_square_release(self):
        message = {"cmd_type":"joystick","Joystick_square":0}
        self.send_message(message)

    def on_left_arrow_press(self):
        message = {"cmd_type":"joystick","Joystick_left":1}
        self.send_message(message)

    def on_left_arrow_release(self):
        message = {"cmd_type":"joystick","Joystick_left":0}
        self.send_message(message)

    def on_right_arrow_press(self):
        message = {"cmd_type":"joystick","Joystick_right":1}
        self.send_message(message)

    def on_right_arrow_release(self):
        message = {"cmd_type":"joystick","Joystick_right":0}
        self.send_message(message)

    def on_up_arrow_press(self):
        message = {"cmd_type":"joystick","Joystick_up":1}
        self.send_message(message)

    def on_up_arrow_release(self):
        message = {"cmd_type":"joystick","Joystick_up":0}
        self.send_message(message)

    def on_down_arrow_press(self):
        message = {"cmd_type":"joystick","Joystick_down":1}
        self.send_message(message)

    def on_down_arrow_release(self):
        message = {"cmd_type":"joystick","Joystick_down":0}
        self.send_message(message)



    # Simple logging functions replacing rclpy.node.Node's get_logger()
    def log_info(self, message):
        print(f"[INFO] {time.strftime('%Y-%m-%d %H:%M:%S')} {message}")

    def log_error(self, message):
        print(f"[ERROR] {time.strftime('%Y-%m-%d %H:%M:%S')} {message}")

    def log_warning(self, message):
        print(f"[WARNING] {time.strftime('%Y-%m-%d %H:%M:%S')} {message}")

    def log_debug(self, message):
        # You might want to control debug output with a flag
        # print(f"[DEBUG] {time.strftime('%Y-%m-%d %H:%M:%S')} {message}")
        pass



    def connect_to_server(self):
        # Keep retrying connection  
        while True: # Changed from rclpy.ok() to an infinite loop for standalone script
            try:
                self.client_socket.connect((SERVER_HOST, SERVER_PORT))
                self.log_info(f"Connected to server at {SERVER_HOST}:{SERVER_PORT}")
                break
            except socket.error as e:
                self.log_error(f"Connection failed: {e}. Retrying in 1 second...")
                time.sleep(1)

    def run(self):
        while True:
            try:
                self.client_socket.connect((SERVER_HOST, SERVER_PORT))
                self.log_info(f"Connected to server at {SERVER_HOST}:{SERVER_PORT}")
                break
            except socket.error as e:
                self.log_error(f"Connection failed: {e}. Retrying in 1 second...")
                time.sleep(1)

    def send_message(self, message):

        try:
            message = json.dumps(message) + '\n'
            self.client_socket.sendall(message.encode('utf-8'))
            self.log_info(f'Sent message: {message.strip()}') 
        except socket.error as e:
            self.log_error(f"Failed to send data: {e}. Attempting to reconnect...") 
            self.client_socket.close()
            self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.connect_to_server()




#events usable:
# on_x_press
# on_x_release
# on_triangle_press
# on_triangle_release
# on_circle_press
# on_circle_release
# on_square_press
# on_square_release
# on_L1_press
# on_L1_release
# on_L2_press
# on_L2_release
# on_R1_press
# on_R1_release
# on_R2_press
# on_R2_release
# on_up_arrow_press
# on_up_down_arrow_release
# on_down_arrow_press
# on_left_arrow_press
# on_left_right_arrow_release
# on_right_arrow_press
# on_L3_up
# on_L3_down
# on_L3_left
# on_L3_right
# on_L3_x_at_rest  # L3 joystick is at rest after the joystick was moved and let go off on x axis
# on_L3_y_at_rest  # L3 joystick is at rest after the joystick was moved and let go off on y axis
# on_L3_press  # L3 joystick is clicked. This event is only detected when connecting without ds4drv
# on_L3_release  # L3 joystick is released after the click. This event is only detected when connecting without ds4drv
# on_R3_up
# on_R3_down
# on_R3_left
# on_R3_right
# on_R3_x_at_rest  # R3 joystick is at rest after the joystick was moved and let go off on x axis
# on_R3_y_at_rest  # R3 joystick is at rest after the joystick was moved and let go off on y axis
# on_R3_press  # R3 joystick is clicked. This event is only detected when connecting without ds4drv
# on_R3_release  # R3 joystick is released after the click. This event is only detected when connecting without ds4drv
# on_options_press
# on_options_release
# on_share_press  # this event is only detected when connecting without ds4drv
# on_share_release  # this event is only detected when connecting without ds4drv
# on_playstation_button_press  # this event is only detected when connecting without ds4drv
# on_playstation_button_release  # this event is only detected when connecting without ds4drv








def main():


    controller = MyController(interface="/dev/input/js0", connecting_using_ds4drv=False)
    # you can start listening before controller is paired, as long as you pair it within the timeout window
    controller.listen(timeout=60)



    try:
        controller.run()
    except KeyboardInterrupt:
        controller.log_info("Script interrupted by user.")
    except Exception as e:
        controller.log_error(f"Unexpected error: {e}")
    finally:
        controller.destroy()

if __name__ == "__main__":
    main()




