#!/usr/bin/env python3
"""Simple Pick-and-Place Demo for AR4 Robot.

This demo performs a basic pick-and-place operation with hardcoded positions.
No vision system required - uses predefined object locations.

Usage:
    ros2 run ar4_pick_and_place_examples simple_pick_place_demo

    Or with custom parameters:
    ros2 run ar4_pick_and_place_examples simple_pick_place_demo --ros-args -p pick_x:=0.3
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Quaternion
import time
import math

# Import our controllers
from ar4_pick_and_place_examples.pick_place_controller import PickPlaceController
from ar4_pick_and_place_examples.gripper_controller import GripperController


class SimplePickPlaceDemo(Node):
    """Simple pick-and-place demonstration with hardcoded positions."""

    def __init__(self):
        """Initialize the demo node."""
        super().__init__("simple_pick_place_demo")

        # Declare parameters for object positions
        self.declare_parameter("pick_x", 0.3)
        self.declare_parameter("pick_y", 0.1)
        self.declare_parameter("pick_z", 0.025)  # Half of 5cm cube height

        self.declare_parameter("place_x", 0.3)
        self.declare_parameter("place_y", -0.15)
        self.declare_parameter("place_z", 0.025)

        self.declare_parameter("object_size", [0.05, 0.05, 0.05])
        self.declare_parameter("gripper_mode", "simulated")

        # Get parameters
        self.pick_pos = [
            self.get_parameter("pick_x").value,
            self.get_parameter("pick_y").value,
            self.get_parameter("pick_z").value,
        ]

        self.place_pos = [
            self.get_parameter("place_x").value,
            self.get_parameter("place_y").value,
            self.get_parameter("place_z").value,
        ]

        self.object_size = self.get_parameter("object_size").value
        self.gripper_mode = self.get_parameter("gripper_mode").value

        self.get_logger().info("=" * 60)
        self.get_logger().info("Simple Pick-and-Place Demo Starting")
        self.get_logger().info("=" * 60)
        self.get_logger().info(
            f"Pick position: ({self.pick_pos[0]:.3f}, {self.pick_pos[1]:.3f}, {self.pick_pos[2]:.3f})"
        )
        self.get_logger().info(
            f"Place position: ({self.place_pos[0]:.3f}, {self.place_pos[1]:.3f}, {self.place_pos[2]:.3f})"
        )
        self.get_logger().info(f"Object size: {self.object_size}")
        self.get_logger().info(f"Gripper mode: {self.gripper_mode}")
        self.get_logger().info("=" * 60)

        # Initialize controllers
        self.get_logger().info("Initializing controllers...")
        self.pick_place_controller = None
        self.gripper_controller = None

    def initialize_controllers(self):
        """Initialize the pick-place and gripper controllers."""
        try:
            self.get_logger().info("Creating PickPlaceController...")
            self.pick_place_controller = PickPlaceController(
                node_name="pick_place_ctrl_demo"
            )

            self.get_logger().info("Creating GripperController...")
            self.gripper_controller = GripperController(
                mode=self.gripper_mode, node_name="gripper_ctrl_demo"
            )

            self.get_logger().info("Controllers initialized successfully!")
            return True

        except Exception as e:
            self.get_logger().error(f"Failed to initialize controllers: {e}")
            return False

    def create_pose(self, x, y, z, orientation="down"):
        """Create a Pose message with given position and orientation.

        Args:
            x, y, z: Position coordinates in meters
            orientation: 'down' for downward-facing gripper

        Returns:
            Pose message
        """
        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z

        if orientation == "down":
            # Gripper pointing downward (rotate 180° around y-axis)
            # Quaternion for 180° rotation around Y: (0, 1, 0, 0)
            # But we might need slight adjustment
            pose.orientation.x = 1.0
            pose.orientation.y = 0.0
            pose.orientation.z = 0.0
            pose.orientation.w = 0.0
        else:
            # Default orientation
            pose.orientation.w = 1.0

        return pose

    def run_demo(self):
        """Execute the complete pick-and-place sequence."""
        if self.pick_place_controller is None or self.gripper_controller is None:
            self.get_logger().error("Controllers not initialized!")
            return False

        try:
            # Step 1: Move to home position
            self.get_logger().info("\n" + "=" * 60)
            self.get_logger().info("STEP 1: Moving to home position")
            self.get_logger().info("=" * 60)
            if not self.pick_place_controller.move_to_home():
                self.get_logger().error("Failed to move to home")
                return False
            time.sleep(1.0)

            # Step 2: Open gripper
            self.get_logger().info("\n" + "=" * 60)
            self.get_logger().info("STEP 2: Opening gripper")
            self.get_logger().info("=" * 60)
            self.gripper_controller.open_gripper()
            time.sleep(0.5)

            # Step 3: Move to pre-grasp position (approach)
            self.get_logger().info("\n" + "=" * 60)
            self.get_logger().info("STEP 3: Moving to approach position (above object)")
            self.get_logger().info("=" * 60)
            pick_pose = self.create_pose(*self.pick_pos, orientation="down")
            approach_pose = self.pick_place_controller.compute_approach_pose(pick_pose)

            if not self.pick_place_controller.move_to_pose(approach_pose):
                self.get_logger().error("Failed to move to approach position")
                return False
            time.sleep(1.0)

            # Step 4: Move down to grasp position
            self.get_logger().info("\n" + "=" * 60)
            self.get_logger().info("STEP 4: Moving down to grasp object")
            self.get_logger().info("=" * 60)
            if not self.pick_place_controller.move_to_pose(pick_pose):
                self.get_logger().error("Failed to move to grasp position")
                return False
            time.sleep(1.0)

            # Step 5: Close gripper and attach object
            self.get_logger().info("\n" + "=" * 60)
            self.get_logger().info("STEP 5: Grasping object")
            self.get_logger().info("=" * 60)
            self.gripper_controller.close_gripper()
            time.sleep(0.5)

            # Attach object to gripper (simulated mode)
            self.gripper_controller.attach_object(
                "pick_object", object_size=self.object_size
            )
            time.sleep(1.0)

            # Step 6: Lift object (retreat)
            self.get_logger().info("\n" + "=" * 60)
            self.get_logger().info("STEP 6: Lifting object")
            self.get_logger().info("=" * 60)
            retreat_pose = self.pick_place_controller.compute_retreat_pose(pick_pose)
            if not self.pick_place_controller.move_to_pose(retreat_pose):
                self.get_logger().error("Failed to lift object")
                return False
            time.sleep(1.0)

            # Step 7: Move to pre-place position
            self.get_logger().info("\n" + "=" * 60)
            self.get_logger().info("STEP 7: Moving to place approach position")
            self.get_logger().info("=" * 60)
            place_pose = self.create_pose(*self.place_pos, orientation="down")
            place_approach = self.pick_place_controller.compute_approach_pose(
                place_pose
            )

            if not self.pick_place_controller.move_to_pose(place_approach):
                self.get_logger().error("Failed to move to place approach")
                return False
            time.sleep(1.0)

            # Step 8: Move down to place position
            self.get_logger().info("\n" + "=" * 60)
            self.get_logger().info("STEP 8: Moving down to place object")
            self.get_logger().info("=" * 60)
            if not self.pick_place_controller.move_to_pose(place_pose):
                self.get_logger().error("Failed to move to place position")
                return False
            time.sleep(1.0)

            # Step 9: Open gripper and detach object
            self.get_logger().info("\n" + "=" * 60)
            self.get_logger().info("STEP 9: Releasing object")
            self.get_logger().info("=" * 60)
            self.gripper_controller.detach_object("pick_object")
            time.sleep(0.5)
            self.gripper_controller.open_gripper()
            time.sleep(1.0)

            # Step 10: Retreat from object
            self.get_logger().info("\n" + "=" * 60)
            self.get_logger().info("STEP 10: Retreating from placed object")
            self.get_logger().info("=" * 60)
            place_retreat = self.pick_place_controller.compute_retreat_pose(place_pose)
            if not self.pick_place_controller.move_to_pose(place_retreat):
                self.get_logger().error("Failed to retreat from place")
                return False
            time.sleep(1.0)

            # Step 11: Return to home
            self.get_logger().info("\n" + "=" * 60)
            self.get_logger().info("STEP 11: Returning to home position")
            self.get_logger().info("=" * 60)
            if not self.pick_place_controller.move_to_home():
                self.get_logger().error("Failed to return home")
                return False

            # Success!
            self.get_logger().info("\n" + "=" * 60)
            self.get_logger().info("✓ DEMO COMPLETED SUCCESSFULLY!")
            self.get_logger().info("=" * 60)

            return True

        except Exception as e:
            self.get_logger().error(f"Demo failed with exception: {e}")
            import traceback

            traceback.print_exc()
            return False


def main(args=None):
    """Main entry point for the demo."""
    rclpy.init(args=args)

    try:
        # Create demo node
        demo = SimplePickPlaceDemo()

        # Initialize controllers
        if not demo.initialize_controllers():
            demo.get_logger().error("Failed to initialize. Exiting.")
            return

        # Give time for everything to stabilize
        demo.get_logger().info("Waiting 2 seconds for initialization...")
        time.sleep(2.0)

        # Run the demo
        success = demo.run_demo()

        if success:
            demo.get_logger().info("\n✓ Demo completed successfully!")
            demo.get_logger().info(
                "The robot should have moved the object from pick to place position."
            )
        else:
            demo.get_logger().error("\n✗ Demo failed! Check logs above for details.")

    except KeyboardInterrupt:
        print("\nDemo interrupted by user")
    except Exception as e:
        print(f"\nDemo failed with exception: {e}")
        import traceback

        traceback.print_exc()
    finally:
        # Cleanup
        rclpy.shutdown()


if __name__ == "__main__":
    main()
