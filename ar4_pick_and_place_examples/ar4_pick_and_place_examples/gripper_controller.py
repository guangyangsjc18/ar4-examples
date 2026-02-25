#!/usr/bin/env python3
"""Gripper Controller for AR4 robot.

Provides abstraction layer supporting both simulated attachment
(using MoveIt's AttachedCollisionObject) and physical gripper control.
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import GripperCommand
from moveit_msgs.msg import AttachedCollisionObject, CollisionObject
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose
import time
from typing import Optional


class GripperController(Node):
    """Controller for gripper operations with simulated and physical modes."""

    def __init__(self, mode="simulated", node_name="gripper_controller"):
        """Initialize the gripper controller.

        Args:
            mode: 'simulated' for attachment simulation or 'physical' for hardware
            node_name: Name of the ROS 2 node
        """
        super().__init__(node_name)

        self.mode = mode
        self.get_logger().info(f"Initializing Gripper Controller in {mode} mode")

        # Declare parameters
        self.declare_parameter("end_effector_link", "link_6")
        self.declare_parameter("gripper_open_position", 0.04)
        self.declare_parameter("gripper_close_position", 0.0)
        self.declare_parameter("gripper_max_effort", 10.0)

        self.end_effector_link = self.get_parameter("end_effector_link").value
        self.gripper_open_pos = self.get_parameter("gripper_open_position").value
        self.gripper_close_pos = self.get_parameter("gripper_close_position").value
        self.gripper_max_effort = self.get_parameter("gripper_max_effort").value

        # Track attached object
        self.attached_object_id = None
        self.is_gripper_open = True

        if self.mode == "simulated":
            self._setup_simulated_mode()
        elif self.mode == "physical":
            self._setup_physical_mode()
        else:
            self.get_logger().error(f"Invalid gripper mode: {mode}")
            raise ValueError(f"Mode must be 'simulated' or 'physical', got '{mode}'")

        self.get_logger().info("Gripper Controller initialized successfully")

    def _setup_simulated_mode(self):
        """Setup for simulated gripper mode."""
        self.get_logger().info("Setting up simulated gripper mode...")

        # Publisher for attached collision objects
        self.attached_object_pub = self.create_publisher(
            AttachedCollisionObject, "/attached_collision_object", 10
        )

        # Publisher for collision objects (for detachment)
        self.collision_object_pub = self.create_publisher(
            CollisionObject, "/collision_object", 10
        )

        time.sleep(0.5)  # Allow publishers to connect
        self.get_logger().info("Simulated mode ready")

    def _setup_physical_mode(self):
        """Setup for physical gripper mode."""
        self.get_logger().info("Setting up physical gripper mode...")

        # Action client for gripper command
        self._gripper_action_client = ActionClient(
            self, GripperCommand, "/gripper_controller/gripper_cmd"
        )

        self.get_logger().info("Waiting for gripper action server...")
        if not self._gripper_action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().warn("Gripper action server not available!")
        else:
            self.get_logger().info("Physical mode ready")

    def open_gripper(self) -> bool:
        """Open the gripper.

        Returns:
            True if successful, False otherwise
        """
        self.get_logger().info("Opening gripper...")

        if self.mode == "simulated":
            # In simulated mode, just track state
            self.is_gripper_open = True
            self.get_logger().info("Gripper opened (simulated)")
            return True
        elif self.mode == "physical":
            return self._send_gripper_command_physical(self.gripper_open_pos)

        return False

    def close_gripper(self) -> bool:
        """Close the gripper.

        Returns:
            True if successful, False otherwise
        """
        self.get_logger().info("Closing gripper...")

        if self.mode == "simulated":
            # In simulated mode, just track state
            self.is_gripper_open = False
            self.get_logger().info("Gripper closed (simulated)")
            return True
        elif self.mode == "physical":
            return self._send_gripper_command_physical(self.gripper_close_pos)

        return False

    def attach_object(
        self,
        object_id: str,
        object_size: Optional[list] = None,
        link_name: Optional[str] = None,
    ) -> bool:
        """Attach an object to the end effector.

        Args:
            object_id: ID of the object to attach
            object_size: Size of object [x, y, z] in meters (for simulated mode)
            link_name: Link to attach to (defaults to end_effector_link)

        Returns:
            True if successful, False otherwise
        """
        if link_name is None:
            link_name = self.end_effector_link

        if object_size is None:
            object_size = [0.05, 0.05, 0.05]  # Default cube size

        self.get_logger().info(f"Attaching object '{object_id}' to {link_name}")

        if self.mode == "simulated":
            return self._attach_object_simulated(object_id, object_size, link_name)
        elif self.mode == "physical":
            # For physical mode, we might add the object to planning scene
            # but not actually command the gripper (that's done by close_gripper)
            self.get_logger().info(f"Object '{object_id}' marked as attached")
            self.attached_object_id = object_id
            return True

        return False

    def detach_object(self, object_id: str) -> bool:
        """Detach an object from the end effector.

        Args:
            object_id: ID of the object to detach

        Returns:
            True if successful, False otherwise
        """
        self.get_logger().info(f"Detaching object '{object_id}'")

        if self.mode == "simulated":
            return self._detach_object_simulated(object_id)
        elif self.mode == "physical":
            self.get_logger().info(f"Object '{object_id}' marked as detached")
            self.attached_object_id = None
            return True

        return False

    def is_object_attached(self) -> bool:
        """Check if an object is currently attached.

        Returns:
            True if object attached, False otherwise
        """
        return self.attached_object_id is not None

    def _attach_object_simulated(
        self, object_id: str, object_size: list, link_name: str
    ) -> bool:
        """Attach object in simulated mode using MoveIt.

        Args:
            object_id: Object ID
            object_size: Object dimensions [x, y, z]
            link_name: Link to attach to

        Returns:
            True if successful
        """
        # Create attached collision object message
        aco = AttachedCollisionObject()
        aco.link_name = link_name
        aco.object.id = object_id
        aco.object.header.frame_id = link_name

        # Define object shape (box)
        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.BOX
        primitive.dimensions = object_size

        # Define object pose (at gripper)
        pose = Pose()
        pose.position.x = 0.0
        pose.position.y = 0.0
        pose.position.z = 0.0
        pose.orientation.w = 1.0

        aco.object.primitives.append(primitive)
        aco.object.primitive_poses.append(pose)

        # Operation: ADD (attach)
        aco.object.operation = CollisionObject.ADD

        # Publish attachment
        self.attached_object_pub.publish(aco)
        time.sleep(0.5)  # Allow time for MoveIt to process

        self.attached_object_id = object_id
        self.get_logger().info(
            f"Object '{object_id}' attached successfully (simulated)"
        )

        return True

    def _detach_object_simulated(self, object_id: str) -> bool:
        """Detach object in simulated mode using MoveIt.

        Args:
            object_id: Object ID

        Returns:
            True if successful
        """
        # Create attached collision object message for removal
        aco = AttachedCollisionObject()
        aco.link_name = self.end_effector_link
        aco.object.id = object_id
        aco.object.operation = CollisionObject.REMOVE

        # Publish detachment
        self.attached_object_pub.publish(aco)
        time.sleep(0.5)  # Allow time for MoveIt to process

        self.attached_object_id = None
        self.get_logger().info(
            f"Object '{object_id}' detached successfully (simulated)"
        )

        return True

    def _send_gripper_command_physical(self, position: float) -> bool:
        """Send command to physical gripper.

        Args:
            position: Target gripper position

        Returns:
            True if successful
        """
        if not self._gripper_action_client.server_is_ready():
            self.get_logger().error("Gripper action server not available")
            return False

        # Create gripper command goal
        goal = GripperCommand.Goal()
        goal.command.position = position
        goal.command.max_effort = self.gripper_max_effort

        self.get_logger().info(f"Sending gripper command: position={position}")

        # Send goal
        send_goal_future = self._gripper_action_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_goal_future)

        goal_handle = send_goal_future.result()

        if not goal_handle.accepted:
            self.get_logger().error("Gripper command rejected")
            return False

        # Wait for result
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        result = result_future.result()

        if result.status == 4:  # SUCCEEDED
            self.get_logger().info("Gripper command completed successfully")
            self.is_gripper_open = position >= self.gripper_open_pos / 2.0
            return True
        else:
            self.get_logger().warn(
                f"Gripper command failed with status: {result.status}"
            )
            return False


def main(args=None):
    """Main function for testing."""
    rclpy.init(args=args)

    try:
        gripper = GripperController(mode="simulated")

        # Test gripper operations
        gripper.get_logger().info("Testing gripper operations...")

        # Open gripper
        gripper.open_gripper()
        time.sleep(1.0)

        # Close gripper
        gripper.close_gripper()
        time.sleep(1.0)

        # Attach object
        gripper.attach_object("test_object", object_size=[0.05, 0.05, 0.05])
        time.sleep(1.0)

        # Check if attached
        if gripper.is_object_attached():
            gripper.get_logger().info("Object is attached")

        # Detach object
        gripper.detach_object("test_object")
        time.sleep(1.0)

        gripper.get_logger().info("Gripper test completed")

    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
