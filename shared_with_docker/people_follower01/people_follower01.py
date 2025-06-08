import cv2
import numpy as np
from ultralytics import YOLO
import time
import serial
import os
import math 

serial_port = '/dev/ttyACM0'
ser = serial.Serial(serial_port, 115200, timeout=1)  # Change port as needed

list_of_servo_to_invert = ["joint0"]
list_of_servo_with_270_degree = ["joint3"]

# Initialize YOLO model
model = YOLO('yolov10x.pt')

# Initialize webcam
cap = cv2.VideoCapture(0)
cap.set(3, 640)
cap.set(4, 480)


if not cap.isOpened():
    print("Error: Could not open webcam.")
    exit()

# Robotic arm control parameters
servo_x_position = 90  # Range: 0-180
servo_y_position = 90  # Range: 0-180
MAX_SERVO_STEP = 15    # Maximum step size for servo movement
CENTER_THRESHOLD = 20  # Pixel threshold to consider object centered
STABILIZATION_DELAY = 0.15  # Seconds to wait after movement

# Movement control variables
last_move_time = 0
processing_allowed = True

# Target classes we're interested in
TARGET_CLASSES = ['person']

def constrain(value, min_val, max_val):
    """Constrain a value between min and max"""
    return max(min_val, min(max_val, value))

def map_value(value, in_min, in_max, out_min, out_max):
    """Map and constrain a value from one range to another"""
    value = constrain(value, in_min, in_max)
    return int((value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min)





def control_servos_with_serial_port(x_pos, y_pos,grabber_position="x"):
    """Send properly scaled servo commands"""
    # Prepare joint commands
    # y_pos = 20
    # joints = [
    #     {"joint0": x_pos},  # X axis
    #     {"joint1": y_pos},  # Y axis
    #     "x", "x", "x", 
    #     {"joint1": 120},
    #     "x", "x"  # Placeholders
    # ]

    joints = [
        {"joint0": x_pos},  # X axis
        {"joint1": "x"},
        {"joint2": "x"},
        {"joint3": "x"},
        {"joint4": "x"},
        {"joint5": "x"},
        {"joint6": "x"},
        {"joint7": "x"},  #add here grabber_position
    ]


    command_parts = []
    
    for joint in joints:
        if joint == "x" or list(joint.values())[0] == "x":
            command_parts.append("x")
            continue
            
        try:
            joint_name = list(joint.keys())[0]
            value = float(list(joint.values())[0])
        except:
            command_parts.append("x")
            continue
        # Apply inversion if needed
        if joint_name in list_of_servo_to_invert:
            value = 180 - value
            
        # Apply special mapping for 270-degree servos
        if joint_name in list_of_servo_with_270_degree:
            value = map_value(value, 0, 180, 0, 270)
        
        command_parts.append(str(int(constrain(value, 0, 270))))
    
    command = ",".join(command_parts) + ";"
    print(f"Sent command: {command}")
    os.system(f'echo "{command}" > {serial_port}')






def open_arm():
    command = "x,x,x,x,x,x,0,20;"
    print(f"Sent command: {command}")
    os.system(f'echo "{command}" > {serial_port}')



def close_arm():
    command = "x,x,x,x,x,x,x,60;"
    print(f"Sent command: {command}")
    os.system(f'echo "{command}" > {serial_port}')


def draw_cross(frame):
    """Draw a red cross dividing the frame into 4 equal parts"""
    height, width = frame.shape[:2]
    center_x, center_y = width // 2, height // 2
    
    # Draw horizontal red line
    cv2.line(frame, (0, center_y), (width, center_y), (0, 0, 255), 2)
    # Draw vertical red line
    cv2.line(frame, (center_x, 0), (center_x, height), (0, 0, 255), 2)
    
    return center_x, center_y

def calculate_proportional_step(distance, frame_size):
    """Calculate proportional servo step based on distance from center"""
    normalized_distance = abs(distance) / (frame_size / 2)
    return int(normalized_distance * MAX_SERVO_STEP)




def move_arm(diff_x, diff_y, frame_width, frame_height,grabber_position="x"):
    """Move arm proportionally based on distance from center"""
    global servo_x_position, servo_y_position, last_move_time, processing_allowed
    
    # Calculate proportional steps
    x_step = calculate_proportional_step(diff_x, frame_width)
    y_step = calculate_proportional_step(diff_y, frame_height)
    
    # Update X position (left/right)
    if abs(diff_x) > CENTER_THRESHOLD:
        if diff_x > 0:
            servo_x_position = min(180, servo_x_position + x_step)
            print(f"Moving RIGHT by {x_step} steps")
        else:
            servo_x_position = max(0, servo_x_position - x_step)
            print(f"Moving LEFT by {x_step} steps")
    
    # Update Y position (up/down)
    if abs(diff_y) > CENTER_THRESHOLD:
        if diff_y > 0:
            servo_y_position = min(180, servo_y_position + y_step)
            print(f"Moving UP by {y_step} steps")
        else:
            servo_y_position = max(0, servo_y_position - y_step)
            print(f"Moving DOWN by {y_step} steps")
    
    print(f"New servo positions - X: {servo_x_position}, Y: {servo_y_position}")
    control_servos_with_serial_port(servo_x_position, "x",grabber_position=grabber_position)
    
    # Set movement time and disable processing temporarily
    last_move_time = time.time()
    processing_allowed = False

def process_frame(frame, center_x, center_y):
    """Process frame with YOLO and calculate object positions"""
    results = model(frame, verbose=False)
    
    found_objects = []
    
    for result in results:
        for box in result.boxes:
            class_id = int(box.cls)
            class_name = model.names[class_id]
            print(f"class_name: {class_name}")  
            confidence = math.ceil((box.conf[0] * 100)) / 100
            if class_id == 0 and confidence > 0.90:  # "person" is class 0 with high confidence
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
                return found_objects  #exit after seeing the first person
    
    return found_objects

def main():
    global servo_x_position, servo_y_position, processing_allowed, last_move_time
    open_arm()
    centered_times = 0
    while True:
        current_time = time.time()
        
        # Check if stabilization period has elapsed
        if not processing_allowed and (current_time - last_move_time) > STABILIZATION_DELAY:
            processing_allowed = True
        
        ret, frame = cap.read()
        if not ret:
            print("Error: Could not read frame.")
            break
        
        # Draw cross and get center coordinates
        center_x, center_y = draw_cross(frame)
        height, width = frame.shape[:2]
        
        # Only process frame if allowed (after stabilization)
        if processing_allowed:
            found_objects = process_frame(frame, center_x, center_y)
            
            # If we found target objects
            if found_objects:
                print("found_objects")
                # Track the first found object with highest confidence
                obj = max(found_objects, key=lambda x: x['confidence'])
                
                # Draw bounding box and center point
                x1, y1, x2, y2 = obj['box']
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.circle(frame, (obj['x'], obj['y']), 5, (0, 255, 0), -1)
                
                # Display distance from center
                cv2.putText(frame, f"X: {obj['diff_x']} Y: {obj['diff_y']}", 
                            (obj['x'] + 10, obj['y'] - 10), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
                
                # Check if we need to move the arm
                if abs(obj['diff_x']) > CENTER_THRESHOLD :   #or abs(obj['diff_y']) > CENTER_THRESHOLD:
                    move_arm(obj['diff_x'], obj['diff_y'], width, height)
                    centered_times = centered_times - 0.2
                    if centered_times < 0:
                        open_arm()
                        centered_times = 0
  
                else:
                    # Object is centered
                    cv2.putText(frame, "CENTERED", (center_x - 60, center_y - 30), 
                                cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                    print("Obj centered")
                    centered_times = centered_times + 1
                    if centered_times > 2:
                        close_arm()
                        centered_times = 2

            else:
                centered_times = 0  
        # Always show the frame (even during stabilization)
        cv2.imshow('Person Tracking', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
