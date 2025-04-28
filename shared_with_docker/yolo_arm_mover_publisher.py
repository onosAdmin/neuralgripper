#!/usr/bin/env python3
import cv2
import numpy as np
from ultralytics import YOLO
import time
import math 
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json

class ArmControlNode(Node):
    def __init__(self):
        super().__init__('arm_control_node')
        self.publisher = self.create_publisher(String, 'arm_commands', 10)
        
        # Initialize YOLO model
        #self.model = YOLO('/shared_with_docker/weights/rock_weights_best4.pt')  
        self.model = YOLO('yolov10x.pt')

        
        # Initialize webcam
        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            self.get_logger().error("Could not open webcam.")
            exit()

        # Robotic arm control parameters
        self.servo_x_position = 90  # Range: 0-180
        self.servo_y_position = 90  # Range: 0-180
        self.MAX_SERVO_STEP = 15    # Maximum step size for servo movement
        self.CENTER_THRESHOLD = 20  # Pixel threshold to consider object centered
        self.STABILIZATION_DELAY = 0.85  # Seconds to wait after movement

        # Movement control variables
        self.last_move_time = 0
        self.processing_allowed = True

        self.get_logger().info('Arm Control Node has been started')

    # def constrain(self, value, min_val, max_val):
    #     """Constrain a value between min and max"""
    #     return max(min_val, min(max_val, value))

    # def map_value(self, value, in_min, in_max, out_min, out_max):
    #     """Map and constrain a value from one range to another"""
    #     value = self.constrain(value, in_min, in_max)
    #     return int((value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min)

    def send_arm_command(self, x_pos, y_pos, grabber_position="x"):
        """Send properly scaled servo commands"""
        command = {
            "x_requested_move": x_pos,
            "y_requested_move": y_pos,
            "grabber_requested_move": grabber_position
        }
        
        msg = String()
        msg.data = json.dumps(command)
        self.publisher.publish(msg)
        self.get_logger().info(f'Sent command: {msg.data}')

    def open_grabber(self):
        self.send_arm_command(self.servo_x_position, self.servo_y_position, 110)

    def close_grabber(self):
        self.send_arm_command(self.servo_x_position, self.servo_y_position, 40)

    def draw_cross(self, frame):
        """Draw a red cross dividing the frame into 4 equal parts"""
        height, width = frame.shape[:2]
        center_x, center_y = width // 2, height // 2
        
        cv2.line(frame, (0, center_y), (width, center_y), (0, 0, 255), 2)
        cv2.line(frame, (center_x, 0), (center_x, height), (0, 0, 255), 2)
        
        return center_x, center_y

    def calculate_proportional_step(self, distance, frame_size):
        """Calculate proportional servo step based on distance from center"""
        normalized_distance = abs(distance) / (frame_size / 2)
        return int(normalized_distance * self.MAX_SERVO_STEP)

    def move_arm(self, diff_x, diff_y, frame_width, frame_height):
        """Move arm proportionally based on distance from center"""
        x_step = self.calculate_proportional_step(diff_x, frame_width)
        y_step = self.calculate_proportional_step(diff_y, frame_height)
        
        if abs(diff_x) > self.CENTER_THRESHOLD:
            if diff_x > 0:
                self.servo_x_position = - x_step # min(180, self.servo_x_position + x_step)
                self.get_logger().info(f"Moving RIGHT by {x_step} steps")
            else:
                self.servo_x_position =  x_step #max(0, self.servo_x_position - x_step)
                self.get_logger().info(f"Moving LEFT by {x_step} steps")
        
        if abs(diff_y) > self.CENTER_THRESHOLD:
            if diff_y > 0:
                self.servo_y_position = min(180, self.servo_y_position + y_step)
                self.get_logger().info(f"Moving UP by {y_step} steps")
            else:
                self.servo_y_position = max(0, self.servo_y_position - y_step)
                self.get_logger().info(f"Moving DOWN by {y_step} steps")
        
        self.get_logger().info(f"New servo positions - X: {self.servo_x_position}, Y: {self.servo_y_position}")
        self.send_arm_command(self.servo_x_position, self.servo_y_position)
        
        self.last_move_time = time.time()
        self.processing_allowed = False

    def process_frame(self, frame, center_x, center_y):
        """Process frame with YOLO and calculate object positions"""
        results = self.model(frame, verbose=False)
        
        found_objects = []
        
        for result in results:
            for box in result.boxes:
                class_id = int(box.cls)
                class_name = self.model.names[class_id]
                confidence = math.ceil((box.conf[0] * 100)) / 100
                if class_id == 0 and confidence > 0.60:  
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
                    return found_objects  # exit after first object is found
        
        return found_objects

    def run(self):
        while rclpy.ok():
            current_time = time.time()
            
            if not self.processing_allowed and (current_time - self.last_move_time) > self.STABILIZATION_DELAY:
                self.processing_allowed = True
            
            ret, frame = self.cap.read()
            if not ret:
                self.get_logger().error("Could not read frame.")
                break
            
            center_x, center_y = self.draw_cross(frame)
            height, width = frame.shape[:2]
            
            if self.processing_allowed:
                found_objects = self.process_frame(frame, center_x, center_y)
                
                if found_objects:
                    obj = max(found_objects, key=lambda x: x['confidence'])
                    
                    x1, y1, x2, y2 = obj['box']
                    cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    cv2.circle(frame, (obj['x'], obj['y']), 5, (0, 255, 0), -1)
                    
                    cv2.putText(frame, f"X: {obj['diff_x']} Y: {obj['diff_y']}", 
                                (obj['x'] + 10, obj['y'] - 10), 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
                    
                    if abs(obj['diff_x']) > self.CENTER_THRESHOLD:
                        self.move_arm(obj['diff_x'], obj['diff_y'], width, height)
                        self.close_grabber()

                    elif abs(obj['diff_y']) > self.CENTER_THRESHOLD:
                        self.move_arm(obj['diff_x'], obj['diff_y'], width, height)
                        self.close_grabber()

                    else:
                        cv2.putText(frame, "CENTERED", (center_x - 60, center_y - 30), 
                                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                        self.open_grabber()

            cv2.imshow('Object Tracking', frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        
        self.cap.release()
        cv2.destroyAllWindows()

def main(args=None):
    rclpy.init(args=args)
    arm_control = ArmControlNode()
    arm_control.run()
    arm_control.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()