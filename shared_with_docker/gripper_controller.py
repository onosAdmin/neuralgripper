#!/usr/bin/env python3

# Handle the gripper rotating the joint6 motor and clamp_moving_joint motor positionsto a given rotation in degrees
# With immediate retry on certain errors and timeout handling

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
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

class MoveGripperToPosition(Node):
    def __init__(self):
        super().__init__('gripper_mover')
        
        # Configure logging
        self.logger = self.get_logger()
        
        # Create the action client for MoveGroup
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
        
        # Initialize variables
        self.current_joint_state = None
        self._planning_success = False
        self._result_received = False
        self._goal_handle = None
        self._timeout_occurred = False
        self._timeout_timer = None
        self._goal_uuid = None
        
        # Wait for the action server to be available
        self.logger.info('Waiting for MoveGroup action server...')
        if not self._action_client.wait_for_server(timeout_sec=10.0):
            self.logger.error('Failed to connect to MoveGroup action server')
            sys.exit(1)
        self.logger.info('MoveGroup action server connected!')

    def joint_state_callback(self, msg):
        """Callback to store the latest joint states."""
        self.current_joint_state = msg
    
    def _timeout_handler(self):
        """Handle timeout for planning and execution"""
        self.logger.error("TIMEOUT: Timeout reached for this attempt")
        self._timeout_occurred = True
        self._result_received = True
        
        # Try to cancel the goal if possible
        if self._goal_handle is not None:
            self.logger.info("Cancelling goal due to timeout...")
            self._goal_handle.cancel_goal_async()
    
    def move_gripper(self, joint6_deg, clamp_deg, max_retries=5, timeout_sec=30.0):
        """
        Move the gripper to the specified positions
        
        Args:
            joint6_deg: Position for joint6 in degrees
            clamp_deg: Position for clamp_moving_joint in degrees (-45 to 45)
            max_retries: Maximum number of planning attempts
            timeout_sec: Timeout in seconds for each attempt
            
        Returns:
            bool: True if movement succeeded, False otherwise
        """
        # Convert degrees to radians
        joint6_rad = degrees_to_radians(joint6_deg)
        clamp_rad = degrees_to_radians(clamp_deg)
        
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
        while attempt < max_retries:
            self.logger.info(f"Planning attempt {attempt+1}/{max_retries}...")
            
            # Reset state for new attempt
            self._result_received = False
            self._planning_success = False
            self._timeout_occurred = False
            
            # Create goal message
            goal_msg = self._create_gripper_goal(joint6_rad, clamp_rad)
            
            # Set up the timeout timer
            self._timeout_timer = threading.Timer(timeout_sec, self._timeout_handler)
            self._timeout_timer.daemon = True
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
                self.logger.info(f"Gripper movement succeeded on attempt {attempt+1} in {elapsed_time:.2f} seconds")
                return True
            
            self.logger.error(f"Attempt {attempt+1} failed after {elapsed_time:.2f} seconds")
            attempt += 1
            
            # If we're going to retry, wait a bit
            if attempt < max_retries:
                self.logger.info(f"Will retry in 2 seconds...")
                time.sleep(2.0)
        
        self.logger.error(f"Gripper movement failed after {max_retries} attempts")
        return False

    def _create_gripper_goal(self, joint6_rad, clamp_rad):
        """Create the MoveGroup goal message for gripper movement"""
        goal_msg = MoveGroup.Goal()
        
        # Set the planning group (should match your MoveIt configuration)
        goal_msg.request.group_name = "gripper_group"  # Make sure this matches your MoveIt setup
        
        # Set workspace parameters
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
        goal_msg.request.allowed_planning_time = 5.0
        goal_msg.request.max_velocity_scaling_factor = 0.5
        goal_msg.request.max_acceleration_scaling_factor = 0.5
        
        # Set the start state to the current state
        goal_msg.request.start_state = RobotState()
        goal_msg.request.start_state.joint_state = self.current_joint_state
        
        # Create joint constraints for just the gripper joints
        joint_constraints = Constraints()
        joint_constraints.name = "gripper_pose"
        
        # Joint names for the gripper (from your controller config)
        gripper_joint_names = ["joint6", "clamp_moving_joint"]
        gripper_positions = [joint6_rad, clamp_rad]
        
        # Add constraints for each gripper joint
        for name, position in zip(gripper_joint_names, gripper_positions):
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

    def get_result_callback(self, future):
        try:
            result = future.result().result
            status_code = result.error_code.val
            
            # Map common error codes to messages
            error_messages = {
                MoveItErrorCodes.SUCCESS: "Gripper movement succeeded!",
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
            
        self._result_received = True

    def wait_for_result(self):
        """Wait for the result of the planning request or until timeout"""
        while not self._result_received and not self._timeout_occurred:
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


def move_gripper_to_position(joint6_deg=0.0, clamp_deg=0.0,init=True):

    if init:
        rclpy.init()
    
    gripper_mover = MoveGripperToPosition()
    motion_result = False
    
    try:
        # Allow node to initialize
        for _ in range(5):
            rclpy.spin_once(gripper_mover, timeout_sec=0.5)

        # Move the gripper
        motion_result = gripper_mover.move_gripper(
            joint6_deg=joint6_deg,
            clamp_deg=clamp_deg,
            max_retries=5,
            timeout_sec=30.0
        )
                
    except KeyboardInterrupt:
        print("\nProgram interrupted by user")
        gripper_mover.cancel_current_execution()
    except Exception as e:
        print(f"\nError occurred: {e}")
        import traceback
        traceback.print_exc()
    finally:
        gripper_mover.destroy_node()
        if init:
            rclpy.shutdown()

    return motion_result


def main(args=None):
    # Example usage:
    print("Opening gripper (joint6=0, clamp=45 degrees)")
    if move_gripper_to_position(joint6_deg=-80, clamp_deg=-90.0):
        print("Gripper opened successfully!")
    else:
        print("Failed to open gripper!")
    
    time.sleep(2.0)
    
    print("\nClosing gripper (joint6=0, clamp=-45 degrees)")
    if move_gripper_to_position(joint6_deg=-80, clamp_deg=160.0):
        print("Gripper closed successfully!")
    else:
        print("Failed to close gripper!")


if __name__ == '__main__':
    main()
