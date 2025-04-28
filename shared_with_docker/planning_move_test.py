#!/usr/bin/env python3

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import Pose, Point, Quaternion
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import MotionPlanRequest, WorkspaceParameters, Constraints, PositionConstraint, OrientationConstraint
from moveit_msgs.msg import RobotState, MoveItErrorCodes
from shape_msgs.msg import SolidPrimitive
from sensor_msgs.msg import JointState
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
import time

class MoveItActionClient(Node):
    def __init__(self):
        super().__init__('moveit_action_client')

        # Create an ActionClient for the MoveGroup action
        self._action_client = ActionClient(self, MoveGroup, 'move_action')

        # Create a subscriber for the current joint states
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,  # Changed from BEST_EFFORT
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.joint_state_subscriber = self.create_subscription(JointState, '/joint_states', self.joint_state_callback, qos_profile)
        self.current_joint_state = None

        # Wait for the MoveGroup action server to be available
        self.get_logger().info("Waiting for MoveGroup action server...")
        self._action_client.wait_for_server()
        self.get_logger().info("MoveGroup action server connected!")

    def joint_state_callback(self, msg):
        """Callback to store the latest joint states."""
        self.current_joint_state = msg

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

        # Set the planner IDime = 20.0

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
        self._send_goal_future = self._action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Goal rejected by server")
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


def main(args=None):
    rclpy.init(args=args)
    moveit_action_client = MoveItActionClient()

    # Define the range for x and the step size
    position_start = -0.18066
    position_end = 0.1
    step_size = 0.001

    # Loop through the x values and send goals
    position = position_start
    while position <= position_end:
        moveit_action_client.get_logger().info(f"Planning for x = {position:.3f}")

        # Reset flags
        moveit_action_client._planning_success = False
        moveit_action_client._result_received = False

        # Send the goal

        moveit_action_client.send_goal(x=0.020354, y=position, z=0.1307902, ox=0.9984, oy=0.056198, oz=-0.00037296, ow=-0.00581)
        # Wait for the result
        while not moveit_action_client._result_received:
            rclpy.spin_once(moveit_action_client, timeout_sec=3)

        # Retry if planning failed
        if not moveit_action_client._planning_success:
            moveit_action_client.get_logger().info("Retrying the request...")
            continue  # Retry the same x value
        print("wait")
        time.sleep(1)  

        
        

        # Proceed to the next step if planning succeeded
        position += step_size

    # Shutdown after completing all steps
    moveit_action_client.get_logger().info("Finished planning for all x values.")
    rclpy.shutdown()


if __name__ == '__main__':
    main()
