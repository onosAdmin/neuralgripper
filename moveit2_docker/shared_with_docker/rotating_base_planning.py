#!/usr/bin/env python3

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import MotionPlanRequest, WorkspaceParameters, Constraints, JointConstraint
from moveit_msgs.msg import RobotState, MoveItErrorCodes
from sensor_msgs.msg import JointState
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
import time
import math

class RotatingBaseController(Node):
    def __init__(self):
        super().__init__('rotating_base_controller')

        # Create an ActionClient for the MoveGroup action
        self._action_client = ActionClient(self, MoveGroup, 'move_action')

        # Create a subscriber for the current joint states
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.joint_state_subscriber = self.create_subscription(
            JointState, 
            '/joint_states', 
            self.joint_state_callback, 
            qos_profile
        )
        self.current_joint_state = None
        self._planning_success = False
        self._result_received = False
        
        # Define all joint names from the URDF
        self.joint_names = [
            'rotating_base',  # This is the one we'll modify
            'joint1',
            'joint2',
            'joint3',
            'joint4',
            'joint5',
            'joint6',
            'clamp_moving_joint'
        ]

        # Wait for the MoveGroup action server to be available
        self.get_logger().info("Waiting for MoveGroup action server...")
        self._action_client.wait_for_server()
        self.get_logger().info("MoveGroup action server connected!")

    def joint_state_callback(self, msg):
        """Callback to store the latest joint states."""
        self.current_joint_state = msg
        
    def get_current_joint_positions(self):
        """Get the current positions of all joints."""
        # Wait for joint states
        while self.current_joint_state is None:
            self.get_logger().info("Waiting for joint state message...")
            rclpy.spin_once(self, timeout_sec=0.5)
        
        # Create a dictionary to store joint positions
        joint_positions = {}
        
        # Extract joint positions from current state
        for joint_name in self.joint_names:
            try:
                joint_index = self.current_joint_state.name.index(joint_name)
                joint_positions[joint_name] = self.current_joint_state.position[joint_index]
                self.get_logger().info(f"Current {joint_name} angle: {joint_positions[joint_name]:.4f} rad ({math.degrees(joint_positions[joint_name]):.2f} degrees)")
            except ValueError:
                self.get_logger().warn(f"Joint '{joint_name}' not found in joint states. Will not constrain this joint.")
                joint_positions[joint_name] = None
        
        return joint_positions

    def send_goal_rotating_base(self, target_angle):
        """
        Send a goal to MoveIt to rotate just the rotating_base joint,
        while keeping all other joints at their current positions.
        """
        while self.current_joint_state is None:
            self.get_logger().info("Waiting for joint state message...")
            rclpy.spin_once(self, timeout_sec=0.5)

        # Get current positions of all joints
        joint_positions = self.get_current_joint_positions()

        # Create a MotionPlanRequest
        goal_msg = MoveGroup.Goal()

        # Set the planning group
        goal_msg.request.group_name = "arm_group"  # Ensure this matches your MoveIt group name

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
        goal_msg.request.start_state.joint_state = self.current_joint_state

        # Set joint constraints for all joints
        goal_constraints = Constraints()
        
        # Add constraint for rotating_base with the target angle
        if joint_positions['rotating_base'] is not None:
            joint_constraint = JointConstraint()
            joint_constraint.joint_name = "rotating_base"
            joint_constraint.position = target_angle  # The new target angle
            joint_constraint.tolerance_above = 0.01  # Small tolerance
            joint_constraint.tolerance_below = 0.01
            joint_constraint.weight = 1.0
            goal_constraints.joint_constraints.append(joint_constraint)
            self.get_logger().info(f"Setting rotating_base target to: {target_angle:.4f} rad ({math.degrees(target_angle):.2f} degrees)")
        
        # Add constraints for all other joints to stay at their current positions
        for joint_name in self.joint_names:
            if joint_name != 'rotating_base' and joint_positions[joint_name] is not None:
                joint_constraint = JointConstraint()
                joint_constraint.joint_name = joint_name
                joint_constraint.position = joint_positions[joint_name]  # Current position
                joint_constraint.tolerance_above = 0.01  # Small tolerance
                joint_constraint.tolerance_below = 0.01
                joint_constraint.weight = 1.0
                goal_constraints.joint_constraints.append(joint_constraint)
                self.get_logger().info(f"Fixing {joint_name} at: {joint_positions[joint_name]:.4f} rad")
        
        goal_msg.request.goal_constraints.append(goal_constraints)

        # Send the goal to the MoveGroup action server
        self.get_logger().info(f"Sending goal to MoveGroup: rotating_base to {target_angle:.4f} rad while keeping other joints fixed")
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
    controller = RotatingBaseController()

    try:
        # Get current joint positions
        joint_positions = controller.get_current_joint_positions()
        current_angle = joint_positions['rotating_base']
        
        if current_angle is None:
            print("Error: Could not determine current rotating_base angle. Exiting.")
            return
        
        # Ask user for rotation parameters
        print("\n=== Rotating Base Joint Controller ===")
        print("This script will rotate ONLY the base joint while keeping all other joints fixed.")
        print(f"Current base angle: {current_angle:.4f} rad ({math.degrees(current_angle):.2f} degrees)")
        print("\nChoose your rotation method:")
        print("1. Absolute angle (set exact position)")
        print("2. Relative angle (move from current position)")
        print("3. Sweep back and forth")
        
        choice = int(input("\nEnter your choice (1-3): "))
        
        if choice == 1:
            # Absolute angle
            target_degrees = float(input("Enter target angle in degrees (-90 to 90): "))
            # Clamp within joint limits
            target_degrees = max(-90, min(90, target_degrees))
            target_angle = math.radians(target_degrees)
            
            # Send the goal
            controller.send_goal_rotating_base(target_angle)
            success = controller.wait_for_result()
            
            if success:
                print(f"Successfully moved to {target_degrees:.2f} degrees")
            else:
                print("Failed to move to target position")
                
        elif choice == 2:
            # Relative angle
            delta_degrees = float(input("Enter relative angle in degrees (+ or -): "))
            current_degrees = math.degrees(current_angle)
            target_degrees = current_degrees + delta_degrees
            
            # Clamp within joint limits
            target_degrees = max(-90, min(90, target_degrees))
            target_angle = math.radians(target_degrees)
            
            # Send the goal
            controller.send_goal_rotating_base(target_angle)
            success = controller.wait_for_result()
            
            if success:
                print(f"Successfully moved to {target_degrees:.2f} degrees")
            else:
                print("Failed to move to target position")
                
        elif choice == 3:
            # Sweep back and forth
            angle_range = float(input("Enter sweep range in degrees (0-90): "))
            angle_range = max(0, min(90, angle_range))
            num_steps = int(input("Enter number of steps per sweep: "))
            num_sweeps = int(input("Enter number of full sweeps: "))
            pause_time = float(input("Enter pause time between steps (seconds): "))
            
            # Calculate center position and step size
            center_angle = current_angle
            half_range_rad = math.radians(angle_range / 2)
            step_size = (2 * half_range_rad) / max(1, num_steps - 1) if num_steps > 1 else 0
            
            print(f"\nStarting sweep around {math.degrees(center_angle):.2f}° with range ±{angle_range/2:.2f}°")
            
            for sweep in range(num_sweeps):
                print(f"\n--- Sweep {sweep+1}/{num_sweeps} ---")
                
                # Forward sweep
                for step in range(num_steps):
                    target_angle = center_angle - half_range_rad + (step * step_size)
                    
                    # Clamp within joint limits (-1.5708 to 1.5708 rad)
                    target_angle = max(-1.5708, min(1.5708, target_angle))
                    
                    print(f"Step {step+1}/{num_steps}: Moving to {math.degrees(target_angle):.2f} degrees")
                    controller.send_goal_rotating_base(target_angle)
                    success = controller.wait_for_result()
                    
                    if not success:
                        print("Failed to move to target position, attempting to continue")
                    
                    # Wait between steps
                    time.sleep(pause_time)
                
                # Get latest joint positions for accurate constraints in the backward sweep
                if sweep < num_sweeps - 1:
                    joint_positions = controller.get_current_joint_positions()
                    current_angle = joint_positions['rotating_base']
                    
                    # Backward sweep (if not the last sweep)
                    for step in range(num_steps):
                        target_angle = center_angle + half_range_rad - (step * step_size)
                        
                        # Clamp within joint limits (-1.5708 to 1.5708 rad)
                        target_angle = max(-1.5708, min(1.5708, target_angle))
                        
                        print(f"Step {step+1}/{num_steps}: Moving to {math.degrees(target_angle):.2f} degrees")
                        controller.send_goal_rotating_base(target_angle)
                        success = controller.wait_for_result()
                        
                        if not success:
                            print("Failed to move to target position, attempting to continue")
                        
                        # Wait between steps
                        time.sleep(pause_time)
                    
                    # Update center angle for next sweep to avoid drift
                    joint_positions = controller.get_current_joint_positions()
                    center_angle = joint_positions['rotating_base']
            
            print("\nSweep movement completed!")
        
        else:
            print("Invalid choice!")
            
    except KeyboardInterrupt:
        print("\nProgram interrupted by user")
    except Exception as e:
        print(f"\nError occurred: {e}")
    finally:
        # Shutdown when done
        controller.get_logger().info("Shutting down...")
        rclpy.shutdown()


if __name__ == '__main__':
    main()
