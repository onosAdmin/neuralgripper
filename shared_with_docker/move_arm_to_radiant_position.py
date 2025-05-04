#!/usr/bin/env python3

# Move the robotic arm to a defined position given the joint_positions in degrees
# With immediate retry on certain errors and 50-second timeout for other cases

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import Pose, Point, Quaternion, TransformStamped
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import MotionPlanRequest, WorkspaceParameters, Constraints, JointConstraint
from moveit_msgs.msg import RobotState, MoveItErrorCodes
from sensor_msgs.msg import JointState
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from rcl_interfaces.msg import Log
import time
import sys
import math
import threading
import re

def degrees_to_radians(degrees):
    """Convert degrees to radians"""
    return degrees * (math.pi / 180.0)

class MoveArmToRadiantPosition(Node):
    def __init__(self):
        super().__init__('robotic_arm_mover')
        
        # Configure logging
        self.logger = self.get_logger()
        
        # Create the action client with correct action name from working example
        self._action_client = ActionClient(self, MoveGroup, 'move_action')
        
        # QoS profile for subscribers
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Subscribe to joint state topic to get current robot state
        self.joint_state_subscriber = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            qos_profile
        )
        
        # Subscribe to logger messages to catch specific errors
        # The correct topic for log messages in ROS2 is /rosout with message type rcl_interfaces/msg/Log
        self.logger_subscriber = self.create_subscription(
            Log,
            '/rosout',
            self.logger_callback,
            qos_profile
        )
        
        # Initialize variables
        self.current_joint_state = None
        self._planning_success = False
        self._result_received = False
        self._goal_handle = None
        self._timeout_occurred = False
        self._timeout_timer = None
        self._immediate_retry_needed = False
        self._goal_uuid = None
        
        # Wait for the action server to be available with increased timeout
        self.logger.info('Waiting for MoveGroup action server...')
        connection_attempts = 0
        max_attempts = 5
        
        while not self._action_client.wait_for_server(timeout_sec=5.0):
            connection_attempts += 1
            if connection_attempts >= max_attempts:
                self.logger.error(f'Failed to connect to MoveGroup action server after {max_attempts} attempts')
                sys.exit(1)
            self.logger.warn(f'MoveGroup action server not available, retrying ({connection_attempts}/{max_attempts})...')
        
        self.logger.info('MoveGroup action server connected!')

    def joint_state_callback(self, msg):
        """Callback to store the latest joint states."""
        self.current_joint_state = msg
    
    def logger_callback(self, msg):
        """Monitor log messages for specific errors that require immediate retry."""
        # Access the message text from the Log message
        message = msg.msg
        
        # Check if this is the specific error we're looking for
        if "Failed to send goal response" in message and "(timeout)" in message and "client will not receive response" in message:
            # Extract the UUID to ensure it's for our goal
            uuid_match = re.search(r'([0-9a-f]{8}-[0-9a-f]{4}-[0-9a-f]{4}-[0-9a-f]{4}-[0-9a-f]{12})', message)
            if uuid_match and self._goal_uuid is not None:
                received_uuid = uuid_match.group(1)
                if received_uuid == self._goal_uuid:
                    self.get_logger().warn(f"Detected goal response failure for our goal: {received_uuid}")
                    self._immediate_retry_needed = True
                    self._result_received = True  # Force the wait loop to exit for immediate retry
    
    def _timeout_handler(self):
        """Handle timeout for planning and execution"""
        self.logger.error("TIMEOUT: 50-second timeout reached for this attempt")
        self._timeout_occurred = True
        self._result_received = True  # Force the wait loop to exit
        
        # Try to cancel the goal if possible
        if self._goal_handle is not None:
            self.logger.info("Cancelling goal due to timeout...")
            self._goal_handle.cancel_goal_async()
    
    def move_to_arm_to_radiant_position(self, joint_positions, joint_names, max_retries=5, max_timeout_sec=50.0):
        """
        Move the robotic arm to the specified joint positions with timeout and immediate retry on specific errors
        
        Args:
            joint_positions: List of joint positions in radians
            joint_names: List of joint names
            max_retries: Maximum number of planning attempts if planning fails
            timeout_sec: Timeout in seconds for each attempt (50 seconds default)
            
        Returns:
            bool: True if movement succeeded, False otherwise
        """
        # Wait for joint state before proceeding with timeout
        wait_start = time.time()
        while self.current_joint_state is None:
            if time.time() - wait_start > 15.0:
                self.logger.error("Timed out waiting for joint state messages")
                return False
                
            self.logger.info("Waiting for joint state message...")
            rclpy.spin_once(self, timeout_sec=0.5)

        # Try planning multiple times if needed
        attempt = 0
        timeout_sec = 10  #first attempt will be done with this timeout
        time.sleep(2)  #wait a bit after starting to ensure the topic is ready
        while attempt < max_retries:
            self.logger.info(f"Planning attempt {attempt+1}/{max_retries} with {timeout_sec}s timeout...")
            
            # Reset state for new attempt
            self._result_received = False
            self._planning_success = False
            self._timeout_occurred = False
            self._immediate_retry_needed = False
            self._goal_uuid = None
            
            # Create goal message
            goal_msg = self._create_goal_message(joint_positions, joint_names)
            
            # Set up the timeout timer
            if attempt > 0:   # after the first  fail the timeout is extended
                timeout_sec = max_timeout_sec
                
            self._timeout_timer = threading.Timer(timeout_sec, self._timeout_handler)
            self._timeout_timer.daemon = True  # Make the timer a daemon so it won't block program exit
            self._timeout_timer.start()
            
            # Send the goal to the MoveGroup action server
            start_time = time.time()
            self._send_goal_future = self._action_client.send_goal_async(goal_msg)
            self._send_goal_future.add_done_callback(self.goal_response_callback)
            
            # Wait for the result
            success = self.wait_for_result()
            
            # Cancel the timeout timer if it's still running
            if self._timeout_timer is not None and self._timeout_timer.is_alive():
                self._timeout_timer.cancel()
                
            elapsed_time = time.time() - start_time
            
            if success:
                self.logger.info(f"Planning succeeded on attempt {attempt+1} in {elapsed_time:.2f} seconds")
                return True
            
            if self._timeout_occurred:
                self.logger.error(f"Attempt {attempt+1} failed due to timeout after {elapsed_time:.2f} seconds")
                # Increment attempt counter for timeout
                attempt += 1
            elif self._immediate_retry_needed:
                self.logger.warn(f"Attempt {attempt+1} failed due to goal response failure - retrying immediately")
                # Don't increment attempt counter for immediate retry cases
                # This means we don't count these as full attempts against our retry limit
                time.sleep(1)  # Small pause to let things reset
            else:
                self.logger.error(f"Attempt {attempt+1} failed after {elapsed_time:.2f} seconds")
                # Increment attempt counter for normal failures
                attempt += 1
                
            # If we're going to retry, wait a bit (but only for normal failures, not immediate retries)
            if attempt < max_retries and not self._immediate_retry_needed:
                self.logger.info(f"Will retry in 2 seconds...")
                time.sleep(2.0)
        
        self.logger.error(f"Planning failed after {max_retries} attempts")
        return False

    def _create_goal_message(self, joint_positions, joint_names):
        """Create the MoveGroup goal message with the given parameters"""
        goal_msg = MoveGroup.Goal()
        
        # Set the planning group
        goal_msg.request.group_name = "arm_group"
        
        # Set the workspace parameters
        goal_msg.request.workspace_parameters = WorkspaceParameters()
        goal_msg.request.workspace_parameters.header.frame_id = "base_link"
        goal_msg.request.workspace_parameters.min_corner.x = -1.0
        goal_msg.request.workspace_parameters.min_corner.y = -1.0
        goal_msg.request.workspace_parameters.min_corner.z = -1.0
        goal_msg.request.workspace_parameters.max_corner.x = 1.0
        goal_msg.request.workspace_parameters.max_corner.y = 1.0
        goal_msg.request.workspace_parameters.max_corner.z = 1.0
        
        # Set planning parameters
        goal_msg.request.num_planning_attempts = 10
        goal_msg.request.allowed_planning_time = 5.0  # Keep this low to allow for multiple quick attempts
        goal_msg.request.max_velocity_scaling_factor = 0.5
        goal_msg.request.max_acceleration_scaling_factor = 0.5
        
        # Set the start state to the current state
        goal_msg.request.start_state = RobotState()
        goal_msg.request.start_state.joint_state = self.current_joint_state
        
        # Create joint constraints
        joint_constraints = Constraints()
        joint_constraints.name = "arm_pose"
        
        # Add a joint constraint for each joint
        for i, (name, position) in enumerate(zip(joint_names, joint_positions)):
            joint_constraint = JointConstraint()
            joint_constraint.joint_name = name
            joint_constraint.position = position
            joint_constraint.tolerance_above = 0.01
            joint_constraint.tolerance_below = 0.01
            joint_constraint.weight = 1.0
            joint_constraints.joint_constraints.append(joint_constraint)
        
        # Add constraints to the goal message
        goal_msg.request.goal_constraints.append(joint_constraints)
        
        return goal_msg

    def goal_response_callback(self, future):
        try:
            goal_handle = future.result()
            # Store UUID for error detection
            if hasattr(goal_handle, 'goal_id') and hasattr(goal_handle.goal_id, 'uuid'):
                self._goal_uuid = str(goal_handle.goal_id.uuid)
                self.logger.info(f"Sent goal with UUID: {self._goal_uuid}")
            
            if not goal_handle.accepted:
                self.logger.error("Goal rejected by server")
                self._result_received = True
                return

            self.logger.info("Goal accepted by server, waiting for result...")
            self._goal_handle = goal_handle
            self._get_result_future = goal_handle.get_result_async()
            self._get_result_future.add_done_callback(self.get_result_callback)
        except Exception as e:
            self.logger.error(f"Exception in goal_response_callback: {e}")
            self._result_received = True
            self._immediate_retry_needed = True

    def get_result_callback(self, future):
        try:
            result = future.result().result
            status_code = result.error_code.val
            
            # Map common error codes to messages
            error_messages = {
                MoveItErrorCodes.SUCCESS: "Planning and execution succeeded!",
                MoveItErrorCodes.PLANNING_FAILED: "Planning failed",
                MoveItErrorCodes.INVALID_MOTION_PLAN: "Invalid motion plan",
                MoveItErrorCodes.MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE: "Plan invalidated by environment change",
                MoveItErrorCodes.CONTROL_FAILED: "Control failed",
                MoveItErrorCodes.UNABLE_TO_AQUIRE_SENSOR_DATA: "Unable to acquire sensor data",
                MoveItErrorCodes.TIMED_OUT: "Planning timed out",
                MoveItErrorCodes.PREEMPTED: "Planning preempted",
            }
            
            message = error_messages.get(status_code, f"Unknown error code: {status_code}")
            
            if status_code == MoveItErrorCodes.SUCCESS:
                self.logger.info(message)
                self._planning_success = True
            else:
                self.logger.error(message)
                self._planning_success = False
        except Exception as e:
            self.logger.error(f"Exception in get_result_callback: {e}")
            self._planning_success = False
            
        # Signal that the result has been received
        self._result_received = True

    def wait_for_result(self):
        """Wait for the result of the planning request or until timeout or other condition triggers."""
        # Spin until we get a result, timeout occurs, or immediate retry is needed
        while not self._result_received and not self._timeout_occurred and not self._immediate_retry_needed:
            rclpy.spin_once(self, timeout_sec=0.1)
        
        return self._planning_success

    def cancel_current_execution(self):
        """Cancel any ongoing execution"""
        if self._goal_handle is not None:
            self.logger.info("Cancelling current goal...")
            self._goal_handle.cancel_goal_async()
        
        # Cancel any timeout timer
        if self._timeout_timer is not None and self._timeout_timer.is_alive():
            self._timeout_timer.cancel()





def move_arm_to_predefined_position(args=None,position="man_scan",joint_positions_list=[]):
    rclpy.init(args=args)
    
    # Create the node
    arm_mover = MoveArmToRadiantPosition()
    motion_result = False
    
    try:
        # Allow node to receive a few callbacks
        for _ in range(5):
            rclpy.spin_once(arm_mover, timeout_sec=0.5)

        #
        if len (joint_positions_list) > 0:

            joint_positions = joint_positions_list


        else:

            position_dict = {
                "small_obj_scan": 
                    # Joint positions expressed in degrees, then converted to radians
                    [
                        degrees_to_radians(0.0),      # rotating_base
                        degrees_to_radians(44.0),      # joint1
                        degrees_to_radians(0.0),      # joint2
                        degrees_to_radians(-90.0),    # joint3
                        degrees_to_radians(0.0),      # joint4
                        degrees_to_radians(-90.0),    # joint5
                    ],
                
                "man_scan": 
                    # Joint positions expressed in degrees, then converted to radians
                    [
                        degrees_to_radians(0.0),      # rotating_base
                        degrees_to_radians(32.0),      # joint1
                        degrees_to_radians(0.0),      # joint2
                        degrees_to_radians(-83.0),    # joint3
                        degrees_to_radians(0.0),      # joint4
                        degrees_to_radians(-61.0),    # joint5
                    ]

                    }
        
            joint_positions = position_dict[position]

        # Joint names
        joint_names = [
            "rotating_base", "joint1", "joint2", "joint3", "joint4", "joint5"
        ]

        # Move to the specified position with retries and smart handling
        max_global_retries = 5
        
        for attempt in range(max_global_retries):
            print(f"\nGlobal attempt {attempt+1}/{max_global_retries}")
            success = arm_mover.move_to_arm_to_radiant_position(
                joint_positions, 
                joint_names, 
                max_retries=6,  # Increased retries since we're not counting immediate retries against the limit
                max_timeout_sec=50.0  # 10-second timeout per attempt
            )
            
            if success:
                print("\nMotion completed successfully!")
                motion_result = True
                break
            elif attempt < max_global_retries - 1:
                print(f"\nFailed on global attempt {attempt+1}, will retry the whole process in 5 seconds...")
                time.sleep(5.0)
            else:
                print("\nFailed to complete motion after all global retry attempts")
                time.sleep(1.0)


                
    except KeyboardInterrupt:
        print("\nProgram interrupted by user")
        # Try to cancel any ongoing execution
        arm_mover.cancel_current_execution()
    except Exception as e:
        print(f"\nError occurred: {e}")
        import traceback
        traceback.print_exc()
    finally:
        # Clean up
        arm_mover.destroy_node()
        rclpy.shutdown()

    arm_mover.destroy_node()
    #rclpy.shutdown()
    return motion_result



def main(args=None):
    motion_result = move_arm_to_predefined_position()
    if motion_result:
        print("\nMotion TO START position completed successfully!")
    else:
        print("\nMotion TO START position failed. Please check the error messages.")


if __name__ == '__main__':
    main()
