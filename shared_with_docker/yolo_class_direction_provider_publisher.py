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

# This node will detect the selected class and will provide direction to center it on the webcam over ros2
# Modified to require 3 sequential detections of the same object in a similar position before sending a command.

class ArmDirectionProvider(Node):
    def __init__(self):
        super().__init__('yolo_data_node')
        String.__import_type_support__()
        self.publisher = self.create_publisher(String, 'yolo_data', 10)




        # Initialize YOLO model
        #self.model = YOLO('/shared_with_docker/weights/rock_weights_best4.pt')
        self.model = YOLO('best_lego_detect.pt')

        # Initialize webcam
        self.cap = cv2.VideoCapture(0)
        self.cap.set(3, 640)
        self.cap.set(4, 480)
        if not self.cap.isOpened():
            self.get_logger().error("Could not open webcam.")
            exit()

        # Robotic arm control parameters
        # Note: These are the requested *moves* relative to current pose, not absolute positions (based on move_arm logic)
        self.servo_x_request = 0 # Will be set based on diff_x sign and magnitude
        self.servo_y_request = 0 # Will be set based on diff_y sign and magnitude

        self.MAX_SERVO_STEP = 15    # Maximum step size for servo movement
        self.CENTER_THRESHOLD = 40  # Pixel threshold to consider object centered
        self.STABILIZATION_DELAY = 0.85  # Seconds to wait after sending a command
        self.confidence_threshold = 0.70

        # Movement control variables
        self.last_move_time = 0
        # self.processing_allowed is now handled by the sequential detection logic primarily
        # We keep it to respect the stabilization delay after a command is sent.
        self.processing_allowed_by_delay = True

        # --- New state variables for sequential detection ---
        self.sequential_count = 0
        # Stores the class name and center coordinates {'class': name, 'x': x, 'y': y}
        # of the object detected in the *previous* frame that contributed to the count.
        self.last_detection_info = None
        # Threshold for considering a new detection in a "similar position"
        self.LOCATION_SIMILARITY_THRESHOLD = 40 # Pixels difference allowed for location

        # --- End new state variables ---


        self.get_logger().info('Arm Control Node has been started')

    def send_arm_direction(self, x_request, y_request, best_current_obj,found_objects, main_object_centered):
        """Send properly scaled servo command requests"""
        

        yolo_data = {
            "x_requested_move": x_request,
            "y_requested_move": y_request,
            "found_objects": str(found_objects), # Send info about all found objects in the last processed frame
            "main_object_centered": main_object_centered,
            "main_target_object": str(best_current_obj)
        }

    

        msg = String()
        msg.data = json.dumps(yolo_data)
        self.publisher.publish(msg)
        self.get_logger().info(f'Sent yolo_data: {msg.data}')

    def draw_cross(self, frame):
        """Draw a red cross dividing the frame into 4 equal parts"""
        height, width = frame.shape[:2]
        center_x, center_y = width // 2, height // 2

        cv2.line(frame, (0, center_y), (width, center_y), (0, 0, 255), 2)
        cv2.line(frame, (center_x, 0), (center_x, height), (0, 0, 255), 2)

        return center_x, center_y

    def calculate_proportional_step(self, distance, frame_size):
        """Calculate proportional servo step based on distance from center"""
        # Use a base step of 1 to ensure small adjustments even close to center
        base_step = 1
        normalized_distance = abs(distance) / (frame_size / 2)
        proportional_step = int(normalized_distance * self.MAX_SERVO_STEP)
        return max(base_step, proportional_step) # Ensure at least base_step

    def move_arm(self, diff_x, diff_y, frame_width, frame_height,best_current_obj,found_objects, main_object_centered):
        """Calculate and send proportional movement requests based on distance from center"""

        x_step = 0
        y_step = 0

        # Determine step size and direction based on difference
        if abs(diff_x) > self.CENTER_THRESHOLD:
             x_step = self.calculate_proportional_step(diff_x, frame_width)
             if diff_x > 0: # Object is to the right of center
                 self.servo_x_request = -x_step # Request to move left (reduce x value typically)
                 self.get_logger().info(f"Requesting LEFT move by {x_step}")
             else: # Object is to the left of center
                 self.servo_x_request = x_step # Request to move right (increase x value typically)
                 self.get_logger().info(f"Requesting RIGHT move by {x_step}")
        else:
             self.servo_x_request = 0 # No X movement needed

        if abs(diff_y) > self.CENTER_THRESHOLD:
             y_step = self.calculate_proportional_step(diff_y, frame_height)
             if diff_y > 0: # Object is below center (positive diff_y with origin top-left)
                 self.servo_y_request = y_step # Request to move DOWN (increase y value typically)
                 self.get_logger().info(f"Requesting DOWN move by {y_step}")
             else: # Object is above center (negative diff_y)
                 self.servo_y_request = -y_step # Request to move UP (decrease y value typically)
                 self.get_logger().info(f"Requesting UP move by {y_step}")
        else:
             self.servo_y_request = 0 # No Y movement needed

        # Only send command if a move is actually requested OR if it just became centered
        if self.servo_x_request != 0 or self.servo_y_request != 0 or main_object_centered == 1:
            self.get_logger().info(f"Requested servo movements - X: {self.servo_x_request}, Y: {self.servo_y_request}")
            self.send_arm_direction(self.servo_x_request, self.servo_y_request,best_current_obj, 0, main_object_centered)

            # Reset delay and state after sending a command based on stable detection
            self.last_move_time = time.time()
            self.processing_allowed_by_delay = False
            # Crucially, reset sequential count and history after a successful move based on stability
            self.sequential_count = 0
            self.last_detection_info = None
            self.get_logger().info("Command sent, resetting sequential count and delay.")
        # else: # No move requested, but object was detected
            # Do not reset count or history here, let it continue building towards stability


    def process_frame(self, frame, center_x, center_y):
        """Process frame with YOLO and calculate object positions"""
        results = self.model(frame, verbose=False)

        found_objects = []

        for result in results:
            # Check if there are boxes in the result
            if result.boxes:
                for box in result.boxes:
                    class_id = int(box.cls)
                    class_name = self.model.names[class_id]
                    confidence = math.ceil((box.conf[0] * 100)) / 100
                    # Filter based on confidence
                    if confidence > self.confidence_threshold:
                        x1, y1, x2, y2 = map(int, box.xyxy[0])
                        obj_x = (x1 + x2) // 2
                        obj_y = (y1 + y2) // 2

                        found_objects.append({
                            'class': class_name,
                            'x': obj_x,
                            'y': obj_y,
                            # Calculate difference from frame center
                            'diff_x': obj_x - center_x,
                            'diff_y': center_y - obj_y, # Note: assuming y=0 is top, positive y is down
                            'box': (x1, y1, x2, y2),
                            'confidence': confidence
                        })

        # Return ALL found objects meeting the criteria
        return found_objects










# Inside the ArmDirectionProvider class, within the run method:

    def run(self):
        # Add state variables (already done in the previous modification)
        self.sequential_count = 0
        self.last_detection_info = None # Stores {'class': name, 'x': x, 'y': y} of the object from the *previous* frame's best detection
        self.LOCATION_SIMILARITY_THRESHOLD = 60 # Pixels difference allowed for location

        # Get the name for class ID 0
        try:
            self.target_class_name = 'brick_2x4' #self.model.names[0]
            self.get_logger().info(f"Target tracking class name: {self.target_class_name} (ID 0)")
        except (IndexError, KeyError):
             self.target_class_name = None
             self.get_logger().warning("Model does not have a class with ID 0. Prioritization of class 0 will not be possible.")


        while rclpy.ok():
            current_time = time.time()

            # Update processing_allowed_by_delay based on time since last move
            if not self.processing_allowed_by_delay and (current_time - self.last_move_time) > self.STABILIZATION_DELAY:
                self.processing_allowed_by_delay = True
                self.get_logger().info("Stabilization delay over. Processing allowed again.")


            ret, frame = self.cap.read()
            if not ret:
                self.get_logger().error("Could not read frame.")
                break

            center_x, center_y = self.draw_cross(frame)
            height, width = frame.shape[:2]

            # Always process the frame to get current detections
            all_found_objects = self.process_frame(frame, center_x, center_y)

            best_current_obj = None
            if all_found_objects:
                # --- Logic to prioritize class 0 ---
                target_class_objects = []
                # Check if target class name was successfully identified and if any objects of that class were found
                if self.target_class_name is not None:
                     target_class_objects = [obj for obj in all_found_objects if obj['class'] == self.target_class_name]

                if target_class_objects:
                    # If target class objects are found, find the best among them (highest confidence)
                    best_current_obj = max(target_class_objects, key=lambda x: x['confidence'])
                    self.get_logger().debug(f"Prioritizing best '{self.target_class_name}' object.")
                else:
                    # If no target class objects (or target_class_name is None), find the best among ALL objects
                    best_current_obj = max(all_found_objects, key=lambda x: x['confidence'])
                    if self.target_class_name is not None:
                         self.get_logger().debug(f"No '{self.target_class_name}' objects found, using best overall object.")
                    else:
                         self.get_logger().debug("Target class name not available, using best overall object.")
                # --- End priority logic ---

                for obj in all_found_objects:
                    x1, y1, x2, y2 = obj['box']
                    # Draw bounding box and info for the best detected object
                    cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    cv2.circle(frame, (obj['x'], obj['y']), 5, (0, 255, 0), -1)
                    cv2.putText(frame, f"{obj['class']} ({obj['confidence']:.2f})",
                                (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
                    cv2.putText(frame, f"Diff X: {obj['diff_x']} Diff Y: {obj['diff_y']}",
                                (obj['x'] + 10, obj['y'] - 30),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)


                x1, y1, x2, y2 = best_current_obj['box']
                # Draw bounding box and info for the best detected object
                cv2.rectangle(frame, (x1, y1), (x2, y2), (255, 0, 255), 2)
                cv2.circle(frame, (best_current_obj['x'], best_current_obj['y']), 5, (255, 0, 0), -1)
                cv2.putText(frame, f"{best_current_obj['class']} ({best_current_obj['confidence']:.2f})",
                        (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1)
                cv2.putText(frame, f"Diff X: {best_current_obj['diff_x']} Diff Y: {best_current_obj['diff_y']}",
                        (best_current_obj['x'] + 10, best_current_obj['y'] - 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1)



                # --- Sequential Detection Logic (This part remains the same from the previous modification) ---
                if best_current_obj:
                    # Check if this detection is similar (same class, similar location) to the last one
                    is_similar_to_last = False
                    if self.last_detection_info:
                        # Check class name
                        class_matches = (best_current_obj['class'] == self.last_detection_info['class'])
                        # Check location similarity
                        loc_diff_x = abs(best_current_obj['x'] - self.last_detection_info['x'])
                        loc_diff_y = abs(best_current_obj['y'] - self.last_detection_info['y'])
                        location_is_similar = (loc_diff_x <= self.LOCATION_SIMILARITY_THRESHOLD and
                                            loc_diff_y <= self.LOCATION_SIMILARITY_THRESHOLD)

                        if class_matches and location_is_similar:
                            is_similar_to_last = True

                    if is_similar_to_last:
                        self.sequential_count += 1
                        self.get_logger().info(f"Sequential count: {self.sequential_count}")
                    else:
                        # New object, different class, or location changed significantly, OR first detection
                        self.sequential_count = 1
                        self.get_logger().info(f"Reset sequential count to 1 (New/Different object detected)")

                    # Store current best detection info for the next frame's comparison
                    self.last_detection_info = {
                        'class': best_current_obj['class'],
                        'x': best_current_obj['x'],
                        'y': best_current_obj['y']
                    }

                    # --- Act if sequential count reaches 3 AND processing is allowed by delay ---
                    if self.sequential_count >= 3 and self.processing_allowed_by_delay:
                        # Object is considered stable over 3 frames in a similar location
                        diff_x = best_current_obj['diff_x']
                        diff_y = best_current_obj['diff_y']

                        main_object_centered = 0
                        if abs(diff_x) <= self.CENTER_THRESHOLD and abs(diff_y) <= self.CENTER_THRESHOLD:
                            main_object_centered = 1
                            cv2.putText(frame, "CENTERED", (center_x - 60, center_y - 30),
                                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                            # If centered, we still call move_arm to potentially send main_object_centered=1
                            # even if requests are 0, or if minor adjustments are needed within threshold logic.
                            self.move_arm(diff_x, diff_y, width, height,best_current_obj, all_found_objects, main_object_centered)
                        else:
                            # Indicate direction if not centered
                            direction_text = ""
                            if abs(diff_x) > self.CENTER_THRESHOLD:
                                direction_text += "LEFT" if diff_x < 0 else "RIGHT"
                                if abs(diff_y) > self.CENTER_THRESHOLD:
                                    direction_text += "/"
                            if abs(diff_y) > self.CENTER_THRESHOLD:
                                # diff_y positive means object is below center (assuming top-left origin)
                                direction_text += "DOWN" if diff_y > 0 else "UP"
                            cv2.putText(frame, direction_text, (center_x - 60, center_y - 30),
                                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                            self.move_arm(diff_x, diff_y, width, height,best_current_obj, all_found_objects, main_object_centered)


                    elif self.sequential_count < 3:
                        # Object detected, but not yet stable for 3 frames
                        # Display progress
                        cv2.putText(frame, f"Stabilizing ({self.sequential_count}/3)",
                                    (center_x - 120, center_y - 30),
                                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 165, 255), 2) # Orange color


            else: # No objects detected in the current frame meeting confidence threshold
                 # Reset count and history if no valid object is seen
                 self.sequential_count = 0
                 self.last_detection_info = None
                 # Display searching status
                 cv2.putText(frame, "Searching...", (center_x - 60, center_y - 30),
                                cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2) # Yellow color
                 #self.move_arm(0, 0, width, height,best_current_obj, all_found_objects, 0)
                 best_current_obj = None
                 self.send_arm_direction(0, 0, 0, all_found_objects, 0)  #no object detected


            # Display stabilization delay status
            if not self.processing_allowed_by_delay:
                 remaining_delay = self.STABILIZATION_DELAY - (current_time - self.last_move_time)
                 cv2.putText(frame, f"Delay: {max(0, remaining_delay):.1f}s",
                            (center_x - 80, center_y + 40),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (128, 128, 128), 2) # Gray color


            cv2.imshow('Object Tracking', frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        # Clean up
        self.cap.release()
        cv2.destroyAllWindows()
        self.get_logger().info("Webcam released and windows closed.")






def main(args=None):
    rclpy.init(args=args)
    arm_direction_provider = ArmDirectionProvider()
    try:
        arm_direction_provider.run()
    except KeyboardInterrupt:
        arm_direction_provider.get_logger().info("Node interrupted by user.")
    finally:
        arm_direction_provider.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
