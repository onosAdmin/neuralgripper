import cv2
import numpy as np
from ultralytics import YOLO
import time
import math
import json
import serial
import os

serial_port = '/dev/ttyACM0'
WRIST_PROXIMITY_THRESHOLD_PERCENT = 5
# Function to calculate the angle between three 2D points (A, B, C) at point B
def calculate_angle(A, B, C):
    # Convert points to numpy arrays
    A = np.array(A)
    B = np.array(B)
    C = np.array(C)

    # Create vectors BA and BC
    BA = A - B
    BC = C - B

    # Calculate dot product
    dot_product = np.dot(BA, BC)

    # Calculate magnitudes
    magnitude_BA = np.linalg.norm(BA)
    magnitude_BC = np.linalg.norm(BC)

    # Avoid division by zero if a magnitude is zero (points are identical)
    if magnitude_BA == 0 or magnitude_BC == 0:
        return 0.0

    # Calculate cosine of the angle
    cosine_angle = dot_product / (magnitude_BA * magnitude_BC)

    # Ensure cosine_angle is within [-1, 1] to prevent numerical errors with arccos
    cosine_angle = np.clip(cosine_angle, -1.0, 1.0)

    # Calculate angle in radians and convert to degrees
    angle_radians = np.arccos(cosine_angle)
    angle_degrees = np.degrees(angle_radians)

    return angle_degrees


def send_cmd_to_serial(msg):
    print(f'Sent command: {msg}')
    os.system('''echo "'''+ msg +'''" > '''+serial_port)



def compose_serial_msg(elbow_angle,gripper_status):
    if gripper_status == "open":
        msg = "0,90,0,"+ str(int(elbow_angle))+",0,90,0,20;"
    else:
        msg = "0,90,0,"+ str(int(elbow_angle))+",0,90,0,89;"
    return msg
    

def calculate_euclidian_distance(point1, point2):
    """
    Calculate the Euclidean distance between two 2D points using basic algebra.
    
    Args:
        point1: First point as [x1, y1]
        point2: Second point as [x2, y2]
        
    Returns:
        The Euclidean distance between the points
    """
    # Extract coordinates
    x1, y1 = point1
    x2, y2 = point2
    
    # Calculate squared differences
    dx = x1 - x2
    dy = y1 - y2
    
    # Calculate squared distance
    distance_squared = dx*dx + dy*dy
    
    # Return the square root (actual distance)
    return math.sqrt(distance_squared)



def main():
    # Load a pre-trained YOLOv11 pose model (e.g., 'yolo11n-pose.pt' for nano version)
    # You can choose larger models like 'yolo11s-pose.pt' or 'yolo11m-pose.pt' for better accuracy if your hardware allows.
    model = YOLO("yolo11n-pose.pt")

    # Open the webcam
    cap = cv2.VideoCapture(0) # 0 for default webcam, change if you have multiple cameras
    img_width = 640
    img_height = 480

    # Set camera resolution to VGA (640x480)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, img_width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, img_height)


    # # Calculate threshold based on percentage of image diagonal
    # image_diagonal = math.sqrt(img_width**2 + img_height**2)
    # wrist_threshold_pixels = (WRIST_PROXIMITY_THRESHOLD_PERCENT / 100.0) * image_diagonal



    
    if not cap.isOpened():
        print("Error: Could not open webcam.")
        return

    while True:
        ret, frame = cap.read()
        if not ret:
            break






        # Perform pose estimation on the frame
        results = model(frame, verbose=False) # verbose=False to suppress detailed output

        # Process results
        for result in results:
            if result.keypoints is not None:
                keypoints_data = result.keypoints.xyn.cpu().numpy() # Normalized keypoints [x, y]
                # If you need pixel coordinates, use result.keypoints.xy
                # keypoints_data = result.keypoints.xy.cpu().numpy()

                # Assuming a single person is detected
                if len(keypoints_data) > 0:
                    person_keypoints = keypoints_data[0] # Get keypoints for the first detected person

                    
                    # Extract the required keypoints (using normalized coordinates for calculation)
                    # Remember the mapping: Right Shoulder (6), Right Elbow (8), Right Wrist (10)
                    try:
                        right_shoulder = person_keypoints[6]
                        right_elbow = person_keypoints[8]
                        right_wrist = person_keypoints[10]
                    except IndexError:
                        print("Not all required keypoints for elbow angle detected.")
                        right_shoulder = [0,0]
                        right_elbow = [0,0]
                        right_wrist = [0,0]
                        

                    try:
                        left_wrist = person_keypoints[9]
                    except IndexError:
                        print("No left wrist detected.")
                        left_wrist = [0,0]


                    gripper_status = "open"
                    # Ensure all keypoints are detected (confidence > 0 or present)
                    if not np.all(right_shoulder == 0) and not np.all(right_elbow == 0) and not np.all(right_wrist == 0):
                        # Calculate the right elbow angle
                        elbow_angle = calculate_angle(right_shoulder, right_elbow, right_wrist)
                        print(f"Right Elbow Angle: {elbow_angle:.2f} degrees")


                        # Check if left and right wrists are close
                        if not np.all(left_wrist == 0):
                            distance_wrists = calculate_euclidian_distance(left_wrist, right_wrist)
                            print(f"Distance between wrists: {distance_wrists:.4f}")
                            WRIST_CLOSENESS_THRESHOLD = 0.12
                            if distance_wrists < WRIST_CLOSENESS_THRESHOLD:
                                print("------------------------------------------")
                                print("!!! LEFT AND RIGHT WRISTS ARE CLOSE !!!")
                                print("------------------------------------------")
                                gripper_status = "closed"
                        else:
                            print("Left or Right wrist not detected for closeness check.")



                        send_cmd_to_serial(compose_serial_msg(elbow_angle,gripper_status))

                        # --- For the wrist, you'd need a third point, e.g., Right Hand (if available) ---
                        # If YOLO-Pose provides a hand/finger tip keypoint (it typically has general wrist only)
                        # you could use it. Otherwise, you might need to infer or estimate a point for the wrist angle.
                        # For demonstration, let's assume we can approximate a hand point relative to the wrist
                        # This is a simplification and may not be accurate for complex wrist movements.
                        
                        # Example for wrist (highly simplified, assumes a straight forearm-hand):
                        # This will give the angle of the wrist joint.
                        # To get the rotation of the wrist, you'd ideally need a 4th point or 3D data.
                        
                        # A better approach for wrist angle might involve projecting a line from elbow through wrist
                        # and comparing it to a line from wrist through a hand point.
                        # For simplicity, let's just calculate a "bend" angle for the wrist
                        # using Elbow-Wrist-FingerTip (if a fingertip keypoint exists or is estimated)
                        
                        # COCO dataset keypoints for hand are usually not as detailed as a full hand model.
                        # If you need detailed hand pose, consider models specialized in hand keypoint detection.
                        
                        # For a simple representation of wrist "flexion/extension" or "deviation",
                        # you might take the line from elbow to wrist, and then a projected line from wrist
                        # to a conceptual "mid-hand" point.
                        
                        # For this example, let's consider the elbow angle only as it's directly calculable
                        # from the provided keypoints (Shoulder, Elbow, Wrist).

                        # If you want a more complex wrist rotation, you'd need to delve into 3D pose or
                        # define a consistent third point for the wrist angle calculation.
                        # For now, we'll just output the elbow.
                    else:
                        print("Not all required keypoints for elbow angle detected.")

                    # Optionally, draw keypoints and lines on the frame for visualization
                    # Ultralytics results objects have built-in plotting capabilities
                    annotated_frame = result.plot()
                    cv2.imshow("YOLO11 Pose Estimation", annotated_frame)
                    
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
