#!/usr/bin/env python3
import cv2
import numpy as np
from ultralytics import YOLO
import time
import math
import json
import socket

# This node will detect the selected class and will provide direction to center it on the webcam over ros2
# Modified to require 3 sequential detections of the same object in a similar position before sending a command.

# --- Socket Client Configuration ---
SERVER_HOST = 'moveit2'  # IP address of the YoloDataSubscriber server
SERVER_PORT = 65432        # Port the YoloDataSubscriber server is listening on
# --- End Socket Client Configuration ---

class ArmDirectionProvider:
    def __init__(self):
        # --- Socket Client Initialization ---
        self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.log_info('Arm Control Node has been started (Pure Socket Client).') 
        self.connect_to_server()
        # --- End Socket Client Initialization ---

        # Initialize YOLO model
        #self.model = YOLO('/shared_with_docker/weights/rock_weights_best4.pt')
        self.model = YOLO('best_lego_detect.pt')

        # Initialize webcam
        self.cap = cv2.VideoCapture(0)
        self.cap.set(3, 640)
        self.cap.set(4, 480)
        if not self.cap.isOpened():
            self.log_error("Could not open webcam.") 
            exit()

        # Robotic arm control parameters
        self.servo_x_request = 0
        self.servo_y_request = 0

        self.MAX_SERVO_STEP = 15
        self.CENTER_THRESHOLD = 40
        self.STABILIZATION_DELAY = 0.85
        self.confidence_threshold = 0.70

        # Movement control variables
        self.last_move_time = 0
        self.processing_allowed_by_delay = True

        # --- New state variables for sequential detection ---
        self.sequential_count = 0
        self.last_detection_info = None
        self.LOCATION_SIMILARITY_THRESHOLD = 40
        # --- End new state variables ---

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

    def send_arm_direction(self, x_request, y_request, best_current_obj, found_objects, main_object_centered):
        """Send properly scaled servo command requests via socket"""

        serializable_best_obj = str(best_current_obj) if best_current_obj is not None else ""
        serializable_found_objects = str(found_objects) if found_objects is not None else "[]"

        yolo_data = {
            "x_requested_move": x_request,
            "y_requested_move": y_request,
            "found_objects": serializable_found_objects,
            "main_object_centered": main_object_centered,
            "main_target_object": serializable_best_obj
        }

        try:
            message = json.dumps(yolo_data) + '\n'
            self.client_socket.sendall(message.encode('utf-8'))
            self.log_info(f'Sent yolo_data: {message.strip()}') 
        except socket.error as e:
            self.log_error(f"Failed to send data: {e}. Attempting to reconnect...") 
            self.client_socket.close()
            self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.connect_to_server()

    def draw_cross(self, frame):
        """Draw a red cross dividing the frame into 4 equal parts"""
        height, width = frame.shape[:2]
        center_x, center_y = width // 2, height // 2

        cv2.line(frame, (0, center_y), (width, center_y), (0, 0, 255), 2)
        cv2.line(frame, (center_x, 0), (center_x, height), (0, 0, 255), 2)

        return center_x, center_y

    def calculate_proportional_step(self, distance, frame_size):
        """Calculate proportional servo step based on distance from center"""
        base_step = 1
        normalized_distance = abs(distance) / (frame_size / 2)
        proportional_step = int(normalized_distance * self.MAX_SERVO_STEP)
        return max(base_step, proportional_step)

    def move_arm(self, diff_x, diff_y, frame_width, frame_height,best_current_obj,found_objects, main_object_centered):
        """Calculate and send proportional movement requests based on distance from center"""

        x_step = 0
        y_step = 0

        if abs(diff_x) > self.CENTER_THRESHOLD:
             x_step = self.calculate_proportional_step(diff_x, frame_width)
             if diff_x > 0:
                 self.servo_x_request = -x_step
                 self.log_info(f"Requesting LEFT move by {x_step}")     
             else:
                 self.servo_x_request = x_step
                 self.log_info(f"Requesting RIGHT move by {x_step}") 
        else:
             self.servo_x_request = 0

        if abs(diff_y) > self.CENTER_THRESHOLD:
             y_step = self.calculate_proportional_step(diff_y, frame_height)
             if diff_y > 0:
                 self.servo_y_request = y_step
                 self.log_info(f"Requesting DOWN move by {y_step}") 
             else:
                 self.servo_y_request = -y_step
                 self.log_info(f"Requesting UP move by {y_step}") 
        else:
             self.servo_y_request = 0

        if self.servo_x_request != 0 or self.servo_y_request != 0 or main_object_centered == 1:
            self.log_info(f"Requested servo movements - X: {self.servo_x_request}, Y: {self.servo_y_request}") 
            self.send_arm_direction(self.servo_x_request, self.servo_y_request,best_current_obj, found_objects, main_object_centered)

            self.last_move_time = time.time()
            self.processing_allowed_by_delay = False
            self.sequential_count = 0
            self.last_detection_info = None
            self.log_info("Command sent, resetting sequential count and delay.") 

    def process_frame(self, frame, center_x, center_y):
        """Process frame with YOLO and calculate object positions"""
        results = self.model(frame, verbose=False)

        found_objects = []

        for result in results:
            if result.boxes:
                for box in result.boxes:
                    class_id = int(box.cls)
                    class_name = self.model.names[class_id]
                    confidence = math.ceil((box.conf[0] * 100)) / 100
                    if confidence > self.confidence_threshold:
                        x1, y1, x2, y2 = map(int, box.xyxy[0])
                        obj_x = (x1 + x2) // 2
                        obj_y = (y1 + y2) // 2

                        found_objects.append({
                            'class': class_name,
                            'x': obj_x,
                            'y': obj_y,
                            'diff_x': obj_x - center_x,
                            'diff_y': center_y - obj_y,
                            'box': (x1, y1, x2, y2),
                            'confidence': confidence
                        })

        return found_objects

    def run(self):
        self.sequential_count = 0
        self.last_detection_info = None
        self.LOCATION_SIMILARITY_THRESHOLD = 60

        try:
            self.target_class_name = 'brick_2x4'
            self.log_info(f"Target tracking class name: {self.target_class_name} (ID 0)") 
        except (IndexError, KeyError):
             self.target_class_name = None
             self.log_warning("Model does not have a class with ID 0. Prioritization of class 0 will not be possible.") 


        while True: # Changed from rclpy.ok() to an infinite loop for standalone script
            current_time = time.time()

            if not self.processing_allowed_by_delay and (current_time - self.last_move_time) > self.STABILIZATION_DELAY:
                self.processing_allowed_by_delay = True
                self.log_info("Stabilization delay over. Processing allowed again.") 


            ret, frame = self.cap.read()
            if not ret:
                self.log_error("Could not read frame.") 
                break

            center_x, center_y = self.draw_cross(frame)
            height, width = frame.shape[:2]

            all_found_objects = self.process_frame(frame, center_x, center_y)

            best_current_obj = None
            if all_found_objects:
                target_class_objects = []
                if self.target_class_name is not None:
                     target_class_objects = [obj for obj in all_found_objects if obj['class'] == self.target_class_name]

                if target_class_objects:
                    best_current_obj = max(target_class_objects, key=lambda x: x['confidence'])
                    self.log_debug(f"Prioritizing best '{self.target_class_name}' object.") 
                else:
                    best_current_obj = max(all_found_objects, key=lambda x: x['confidence'])
                    if self.target_class_name is not None:
                         self.log_debug(f"No '{self.target_class_name}' objects found, using best overall object.") 
                    else:
                         self.log_debug("Target class name not available, using best overall object.") 

                for obj in all_found_objects:
                    x1, y1, x2, y2 = obj['box']
                    cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    cv2.circle(frame, (obj['x'], obj['y']), 5, (0, 255, 0), -1)
                    cv2.putText(frame, f"{obj['class']} ({obj['confidence']:.2f})",
                                (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
                    cv2.putText(frame, f"Diff X: {obj['diff_x']} Diff Y: {obj['diff_y']}",
                                (obj['x'] + 10, obj['y'] - 30),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)


                x1, y1, x2, y2 = best_current_obj['box']
                cv2.rectangle(frame, (x1, y1), (x2, y2), (255, 0, 255), 2)
                cv2.circle(frame, (best_current_obj['x'], best_current_obj['y']), 5, (255, 0, 0), -1)
                cv2.putText(frame, f"{best_current_obj['class']} ({best_current_obj['confidence']:.2f})",
                        (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1)
                cv2.putText(frame, f"Diff X: {best_current_obj['diff_x']} Diff Y: {best_current_obj['diff_y']}",
                        (best_current_obj['x'] + 10, best_current_obj['y'] - 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1)


                if best_current_obj:
                    is_similar_to_last = False
                    if self.last_detection_info:
                        class_matches = (best_current_obj['class'] == self.last_detection_info['class'])
                        loc_diff_x = abs(best_current_obj['x'] - self.last_detection_info['x'])
                        loc_diff_y = abs(best_current_obj['y'] - self.last_detection_info['y'])
                        location_is_similar = (loc_diff_x <= self.LOCATION_SIMILARITY_THRESHOLD and
                                            loc_diff_y <= self.LOCATION_SIMILARITY_THRESHOLD)

                        if class_matches and location_is_similar:
                            is_similar_to_last = True

                    if is_similar_to_last:
                        self.sequential_count += 1
                        self.log_info(f"Sequential count: {self.sequential_count}") 
                    else:
                        self.sequential_count = 1
                        self.log_info(f"Reset sequential count to 1 (New/Different object detected)") 

                    self.last_detection_info = {
                        'class': best_current_obj['class'],
                        'x': best_current_obj['x'],
                        'y': best_current_obj['y']
                    }

                    if self.sequential_count >= 3 and self.processing_allowed_by_delay:
                        diff_x = best_current_obj['diff_x']
                        diff_y = best_current_obj['diff_y']

                        main_object_centered = 0
                        if abs(diff_x) <= self.CENTER_THRESHOLD and abs(diff_y) <= self.CENTER_THRESHOLD:
                            main_object_centered = 1
                            cv2.putText(frame, "CENTERED", (center_x - 60, center_y - 30),
                                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                            self.move_arm(diff_x, diff_y, width, height,best_current_obj, all_found_objects, main_object_centered)
                        else:
                            direction_text = ""
                            if abs(diff_x) > self.CENTER_THRESHOLD:
                                direction_text += "LEFT" if diff_x < 0 else "RIGHT"
                                if abs(diff_y) > self.CENTER_THRESHOLD:
                                    direction_text += "/"
                            if abs(diff_y) > self.CENTER_THRESHOLD:
                                direction_text += "DOWN" if diff_y > 0 else "UP"
                            cv2.putText(frame, direction_text, (center_x - 60, center_y - 30),
                                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                            self.move_arm(diff_x, diff_y, width, height,best_current_obj, all_found_objects, main_object_centered)


                    elif self.sequential_count < 3:
                        cv2.putText(frame, f"Stabilizing ({self.sequential_count}/3)",
                                    (center_x - 120, center_y - 30),
                                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 165, 255), 2)

            else:
                 self.sequential_count = 0
                 self.last_detection_info = None
                 cv2.putText(frame, "Searching...", (center_x - 60, center_y - 30),
                                cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)
                 best_current_obj = None
                 self.send_arm_direction(0, 0, None, [], 0)


            if not self.processing_allowed_by_delay:
                 remaining_delay = self.STABILIZATION_DELAY - (current_time - self.last_move_time)
                 cv2.putText(frame, f"Delay: {max(0, remaining_delay):.1f}s",
                            (center_x - 80, center_y + 40),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (128, 128, 128), 2)


            cv2.imshow('Object Tracking', frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        # Cleanup
        self.cap.release()
        cv2.destroyAllWindows()
        self.log_info("Webcam released and windows closed.") 

    # Simplified destroy method for a non-ROS node
    def destroy(self):
        self.log_info("Closing socket client.") 
        self.client_socket.close()


def main():
    arm_direction_provider = ArmDirectionProvider()
    try:
        arm_direction_provider.run()
    except KeyboardInterrupt:
        arm_direction_provider.log_info("Script interrupted by user.")
    finally:
        arm_direction_provider.destroy()

if __name__ == "__main__":
    main()