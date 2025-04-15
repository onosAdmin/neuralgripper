#!/usr/bin/env python3

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import Pose, Point, Quaternion, TransformStamped
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import MotionPlanRequest, WorkspaceParameters, Constraints, PositionConstraint, OrientationConstraint
from moveit_msgs.msg import RobotState, MoveItErrorCodes
from shape_msgs.msg import SolidPrimitive
from sensor_msgs.msg import JointState
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
import time
import tf2_ros
from tf2_ros import TransformException
import numpy as np
import sys

class MoveItActionClient(Node):
    def __init__(self):
        super().__init__('moveit_action_client')

        # Create an ActionClient for the MoveGroup action
        self._action_client = ActionClient(self, MoveGroup, 'move_action')

        # Create a subscriber for the current joint states
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.joint_state_subscriber = self.create_subscription(JointState, '/joint_states', self.joint_state_callback, qos_profile)
        self.current_joint_state = None
        self._planning_success = False
        self._result_received = False

        # Setup TF2 listener to get link6 position
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Wait for the MoveGroup action server to be available
        self.get_logger().info("Waiting for MoveGroup action server...")
        self._action_client.wait_for_server()
        self.get_logger().info("MoveGroup action server connected!")

    def joint_state_callback(self, msg):
        """Callback to store the latest joint states."""
        self.current_joint_state = msg
        
    def get_current_position(self):
        """
        Get the current position of link6 using TF2.
        Returns a dictionary with x, y, z, ox, oy, oz, ow.
        """
        # Wait for joint states
        while self.current_joint_state is None:
            self.get_logger().info("Waiting for joint state message...")
            rclpy.spin_once(self, timeout_sec=0.5)
        
        # Try to get the transform from base_link to link6
        max_attempts = 10
        attempts = 0
        position = None
        
        while position is None and attempts < max_attempts:
            try:
                # Get the transform from base_link to link6
                transform = self.tf_buffer.lookup_transform(
                    'world',     # Target frame
                    'link6',     # Source frame
                    rclpy.time.Time(),  # Get latest transform
                    rclpy.duration.Duration(seconds=1.0)  # Wait up to 1 second
                )
                
                # Extract position and orientation from the transform
                position = {
                    'x': transform.transform.translation.x,
                    'y': transform.transform.translation.y,
                    'z': transform.transform.translation.z,
                    'ox': transform.transform.rotation.x,
                    'oy': transform.transform.rotation.y,
                    'oz': transform.transform.rotation.z,
                    'ow': transform.transform.rotation.w
                }
                
                self.get_logger().info(f"Current position of link6: X={position['x']:.6f}, Y={position['y']:.6f}, Z={position['z']:.6f}")
                
            except TransformException as ex:
                attempts += 1
                self.get_logger().warn(f"Could not get transform: {ex}. Attempt {attempts}/{max_attempts}")
                rclpy.spin_once(self, timeout_sec=0.1)
        
        if position is None:
            # If we couldn't get the transform, use a default position
            # This is a fallback that could be replaced with more sophisticated logic
            self.get_logger().error("Could not get current position from TF. Using default position.")
            position = {'x': 0.0, 'y': 0.0, 'z': 0.3, 'ox': 0.0, 'oy': 0.0, 'oz': 0.0, 'ow': 1.0}
        
        return position

    def send_goal(self, **kwargs):
        """
        Send a goal to MoveIt with constraints.
        Only the provided variables (x, y, z, ox, oy, oz, ow) will be constrained.
        Omitted variables are treated as indifferent (no constraint applied).
        """
        while self.current_joint_state is None:
            self.get_logger().info("Waiting for joint state message...")
            rclpy.spin_once(self, timeout_sec=0.5)

        # Create a MotionPlanRequest
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
        goal_msg.request.allowed_planning_time = 2.5

        # Set the start state to the actual current state
        goal_msg.request.start_state = RobotState()
        goal_msg.request.start_state.joint_state = self.current_joint_state  # Use the latest joint state

        # Set the goal constraints (target pose)
        goal_constraints = Constraints()

        # Position constraint (only if x, y, or z is provided)
        if any(key in kwargs for key in ['x', 'y', 'z']):
            position_constraint = PositionConstraint()
            position_constraint.header.frame_id = "world"
            position_constraint.link_name = "link6"  # End effector link
            position_constraint.constraint_region.primitives.append(SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[0.01, 0.01, 0.01]))
            position_constraint.constraint_region.primitive_poses.append(Pose(position=Point()))

            # Apply position constraints only for provided variables
            if 'x' in kwargs:
                position_constraint.constraint_region.primitive_poses[0].position.x = kwargs['x']
            if 'y' in kwargs:
                position_constraint.constraint_region.primitive_poses[0].position.y = kwargs['y']
            if 'z' in kwargs:
                position_constraint.constraint_region.primitive_poses[0].position.z = kwargs['z']

            position_constraint.weight = 1.0
            goal_constraints.position_constraints.append(position_constraint)

        # Orientation constraint (only if ox, oy, oz, or ow is provided)
        if any(key in kwargs for key in ['ox', 'oy', 'oz', 'ow']):
            orientation_constraint = OrientationConstraint()
            orientation_constraint.header.frame_id = "base_link"
            orientation_constraint.link_name = "link6"
            orientation_constraint.orientation = Quaternion(
                x=kwargs.get('ox', 0.0),
                y=kwargs.get('oy', 0.0),
                z=kwargs.get('oz', 0.0),
                w=kwargs.get('ow', 1.0)
            )
            orientation_constraint.absolute_x_axis_tolerance = 0.05 if 'ox' in kwargs else float('inf')
            orientation_constraint.absolute_y_axis_tolerance = 0.05 if 'oy' in kwargs else float('inf')
            orientation_constraint.absolute_z_axis_tolerance = 0.05 if 'oz' in kwargs else float('inf')
            orientation_constraint.weight = 1.0
            goal_constraints.orientation_constraints.append(orientation_constraint)

        # Add constraints to the goal message
        if goal_constraints.position_constraints or goal_constraints.orientation_constraints:
            goal_msg.request.goal_constraints.append(goal_constraints)

        # Send the goal to the MoveGroup action server
        self.get_logger().info(f"Sending goal to MoveGroup with constraints: {kwargs}")
        self._result_received = False
        self._planning_success = False
        self._send_goal_future = self._action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Goal rejected by server")
            self._result_received = True
            return

        self.get_logger().info("Goal accepted by server, waiting for result...")
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        if result.error_code.val == MoveItErrorCodes.SUCCESS:
            self.get_logger().info("Planning and execution succeeded!")
            self._planning_success = True
        else:
            self.get_logger().error(f"Planning failed with error code: {result.error_code.val}")
            self._planning_success = False

        # Signal that the result has been received
        self._result_received = True

    def wait_for_result(self, timeout=10.0):
        """Wait for the result of the planning request."""
        start_time = time.time()
        while not self._result_received:
            if time.time() - start_time > timeout:
                self.get_logger().error("Timeout waiting for planning result")
                return False
            rclpy.spin_once(self, timeout_sec=0.1)
        
        return self._planning_success


def main(args=None):
    rclpy.init(args=args)
    moveit_action_client = MoveItActionClient()

    try:
        # Get current position from robot
        position = moveit_action_client.get_current_position()
        
        # Parameters for incremental movement
        axis = 'x'  # We're moving along the X-axis
        step_size = float(input(f"Enter step size for {axis}-axis (e.g., 0.001): "))
        num_steps = int(input("Enter number of steps to perform: "))
        sleep_time = float(input("Enter sleep time between moves (seconds): "))
        
        print(f"\nStarting incremental movement on {axis}-axis")
        print(f"Initial position: X={position['x']:.6f}, Y={position['y']:.6f}, Z={position['z']:.6f}")
        
        # Perform the incremental movement
        for i in range(num_steps):
            # Increment the X axis
            position[axis] += step_size
            
            print(f"\nStep {i+1}/{num_steps}: Moving to {axis}={position[axis]:.6f}")
            
            # Send the goal with all position parameters
            moveit_action_client.send_goal(**position)
            
            # Wait for the result
            success = moveit_action_client.wait_for_result()
            
            if not success:
                print(f"Failed to move to {axis}={position[axis]:.6f}. Retrying...")
                position[axis] -= step_size  # Revert the position change
                retries = 0
                max_retries = 3
                
                while not success and retries < max_retries:
                    retries += 1
                    print(f"Retry attempt {retries}/{max_retries}...")
                    time.sleep(1.0)  # Wait before retrying
                    
                    # Try with current position again
                    position = moveit_action_client.get_current_position()
                    position[axis] += step_size  # Apply increment to current position
                    
                    # Send goal again
                    moveit_action_client.send_goal(**position)
                    success = moveit_action_client.wait_for_result()
                
                if not success:
                    print(f"Failed to continue movement after {max_retries} retry attempts. Stopping.")
                    break
                    
            print(f"Successfully moved to {axis}={position[axis]:.6f}")
            
            # Sleep between movements
            print(f"Waiting {sleep_time} seconds before next movement...")
            time.sleep(sleep_time)
            
            # Update position from robot to avoid drift
            if i < num_steps - 1:  # Don't need to update after the last step
                position = moveit_action_client.get_current_position()
        
        print("\nIncremental movement completed!")
            
    except KeyboardInterrupt:
        print("\nProgram interrupted by user")
    except Exception as e:
        print(f"\nError occurred: {e}")
    finally:
        # Shutdown when done
        moveit_action_client.get_logger().info("Shutting down...")
        rclpy.shutdown()


if __name__ == '__main__':
    main()
