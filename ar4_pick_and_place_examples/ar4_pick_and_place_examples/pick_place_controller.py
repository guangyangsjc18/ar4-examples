#!/usr/bin/env python3
"""Pick and Place Controller for AR4 robot using MoveIt 2.

This module provides a high-level interface for pick-and-place operations
using MoveIt 2 motion planning and execution.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion
from moveit_msgs.msg import CollisionObject, AttachedCollisionObject
from moveit_msgs.srv import GetPositionIK
from shape_msgs.msg import SolidPrimitive
import math
from typing import List, Optional, Tuple
import time


class PickPlaceController(Node):
    """High-level pick-and-place controller using MoveIt 2."""

    def __init__(self, node_name="pick_place_controller"):
        """Initialize the pick-place controller.

        Args:
            node_name: Name of the ROS 2 node
        """
        super().__init__(node_name)

        # Declare parameters
        self.declare_parameter("planning_group", "arm")
        self.declare_parameter("end_effector_link", "link_6")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("planning_time", 10.0)
        self.declare_parameter("velocity_scaling", 0.1)
        self.declare_parameter("acceleration_scaling", 0.1)
        self.declare_parameter("approach_distance", 0.1)
        self.declare_parameter("retreat_distance", 0.1)
        self.declare_parameter("max_planning_attempts", 5)

        # Get parameters
        self.planning_group = self.get_parameter("planning_group").value
        self.end_effector_link = self.get_parameter("end_effector_link").value
        self.base_frame = self.get_parameter("base_frame").value
        self.planning_time = self.get_parameter("planning_time").value
        self.velocity_scaling = self.get_parameter("velocity_scaling").value
        self.acceleration_scaling = self.get_parameter("acceleration_scaling").value
        self.approach_distance = self.get_parameter("approach_distance").value
        self.retreat_distance = self.get_parameter("retreat_distance").value
        self.max_attempts = self.get_parameter("max_planning_attempts").value

        self.get_logger().info(
            f"Initializing Pick-Place Controller for group: {self.planning_group}"
        )

        # Wait for MoveIt to be ready
        self.get_logger().info("Waiting for MoveIt to initialize...")
        time.sleep(2.0)

        # Initialize MoveIt interface
        try:
            from moveit.planning import (
                MoveGroupInterface,
                PlanningSceneInterface,
            )

            self.move_group = MoveGroupInterface(
                node=self,
                group_name=self.planning_group,
                robot_description="robot_description",
            )

            self.planning_scene = PlanningSceneInterface(
                node=self,
                ns="",
            )

            self.get_logger().info("MoveIt interfaces initialized successfully")

        except Exception as e:
            self.get_logger().error(f"Failed to initialize MoveIt: {e}")
            raise

        # Set planning parameters
        self.move_group.set_max_velocity_scaling_factor(self.velocity_scaling)
        self.move_group.set_max_acceleration_scaling_factor(self.acceleration_scaling)
        self.move_group.set_planning_time(self.planning_time)
        self.move_group.set_num_planning_attempts(self.max_attempts)

        self.get_logger().info(
            f"Controller initialized. End effector: {self.end_effector_link}"
        )

        # Store current state
        self.current_attached_object = None

    def move_to_home(self) -> bool:
        """Move robot to home position (all joints at 0).

        Returns:
            True if successful, False otherwise
        """
        self.get_logger().info("Moving to home position...")
        home_joints = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        return self.move_to_joint_positions(home_joints)

    def move_to_joint_positions(self, joint_positions: List[float]) -> bool:
        """Move to specific joint positions.

        Args:
            joint_positions: List of 6 joint angles in radians

        Returns:
            True if successful, False otherwise
        """
        if len(joint_positions) != 6:
            self.get_logger().error(
                f"Expected 6 joint values, got {len(joint_positions)}"
            )
            return False

        self.get_logger().info(
            f"Planning to joint target: {[f'{j:.3f}' for j in joint_positions]}"
        )

        try:
            self.move_group.set_joint_value_target(joint_positions)
            success = self.move_group.move()

            if success:
                self.get_logger().info("Successfully reached joint target")
            else:
                self.get_logger().warn("Failed to reach joint target")

            return success

        except Exception as e:
            self.get_logger().error(f"Exception during joint motion: {e}")
            return False

    def move_to_pose(self, pose: Pose, frame_id: str = None) -> bool:
        """Move end effector to a specific pose.

        Args:
            pose: Target pose for end effector
            frame_id: Reference frame (defaults to base_frame)

        Returns:
            True if successful, False otherwise
        """
        if frame_id is None:
            frame_id = self.base_frame

        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = frame_id
        pose_stamped.header.stamp = self.get_clock().now().to_msg()
        pose_stamped.pose = pose

        self.get_logger().info(
            f"Planning to pose: position=({pose.position.x:.3f}, "
            f"{pose.position.y:.3f}, {pose.position.z:.3f})"
        )

        try:
            self.move_group.set_pose_target(pose_stamped)
            success = self.move_group.move()

            if success:
                self.get_logger().info("Successfully reached pose target")
            else:
                self.get_logger().warn("Failed to reach pose target")

            return success

        except Exception as e:
            self.get_logger().error(f"Exception during pose motion: {e}")
            return False

    def compute_approach_pose(
        self, grasp_pose: Pose, approach_distance: float = None
    ) -> Pose:
        """Compute approach pose above the grasp pose.

        Args:
            grasp_pose: Target grasp pose
            approach_distance: Distance above grasp (uses default if None)

        Returns:
            Approach pose
        """
        if approach_distance is None:
            approach_distance = self.approach_distance

        approach_pose = Pose()
        approach_pose.position.x = grasp_pose.position.x
        approach_pose.position.y = grasp_pose.position.y
        approach_pose.position.z = grasp_pose.position.z + approach_distance
        approach_pose.orientation = grasp_pose.orientation

        return approach_pose

    def compute_retreat_pose(
        self, current_pose: Pose, retreat_distance: float = None
    ) -> Pose:
        """Compute retreat pose above current pose.

        Args:
            current_pose: Current pose
            retreat_distance: Distance to retreat (uses default if None)

        Returns:
            Retreat pose
        """
        if retreat_distance is None:
            retreat_distance = self.retreat_distance

        retreat_pose = Pose()
        retreat_pose.position.x = current_pose.position.x
        retreat_pose.position.y = current_pose.position.y
        retreat_pose.position.z = current_pose.position.z + retreat_distance
        retreat_pose.orientation = current_pose.orientation

        return retreat_pose

    def plan_cartesian_path(self, waypoints: List[Pose], frame_id: str = None) -> bool:
        """Plan and execute a Cartesian path through waypoints.

        Args:
            waypoints: List of poses to move through
            frame_id: Reference frame (defaults to base_frame)

        Returns:
            True if successful, False otherwise
        """
        if frame_id is None:
            frame_id = self.base_frame

        self.get_logger().info(
            f"Planning Cartesian path with {len(waypoints)} waypoints"
        )

        try:
            # Convert to PoseStamped list
            pose_list = []
            for pose in waypoints:
                ps = PoseStamped()
                ps.header.frame_id = frame_id
                ps.header.stamp = self.get_clock().now().to_msg()
                ps.pose = pose
                pose_list.append(ps)

            # Plan Cartesian path
            success = self.move_group.compute_cartesian_path(pose_list, 0.01, 0.0)

            if success:
                self.get_logger().info("Cartesian path planned successfully")
                # Execute the plan
                return self.move_group.execute()
            else:
                self.get_logger().warn("Failed to plan Cartesian path")
                return False

        except Exception as e:
            self.get_logger().error(f"Exception during Cartesian planning: {e}")
            return False

    def get_current_pose(self) -> Optional[PoseStamped]:
        """Get current end effector pose.

        Returns:
            Current pose or None if failed
        """
        try:
            return self.move_group.get_current_pose()
        except Exception as e:
            self.get_logger().error(f"Failed to get current pose: {e}")
            return None

    def get_current_joint_values(self) -> Optional[List[float]]:
        """Get current joint values.

        Returns:
            List of joint values or None if failed
        """
        try:
            return self.move_group.get_current_joint_values()
        except Exception as e:
            self.get_logger().error(f"Failed to get current joint values: {e}")
            return None

    def add_collision_box(
        self, name: str, pose: Pose, size: List[float], frame_id: str = None
    ) -> bool:
        """Add a box collision object to the planning scene.

        Args:
            name: Object name
            pose: Object pose
            size: Box dimensions [x, y, z] in meters
            frame_id: Reference frame (defaults to base_frame)

        Returns:
            True if successful
        """
        if frame_id is None:
            frame_id = self.base_frame

        self.get_logger().info(f"Adding collision box '{name}' to scene")

        try:
            self.planning_scene.add_box(name, pose, size, frame_id)
            time.sleep(0.5)  # Allow time for update
            return True
        except Exception as e:
            self.get_logger().error(f"Failed to add collision box: {e}")
            return False

    def remove_collision_object(self, name: str) -> bool:
        """Remove a collision object from the planning scene.

        Args:
            name: Object name

        Returns:
            True if successful
        """
        self.get_logger().info(f"Removing collision object '{name}'")

        try:
            self.planning_scene.remove_object(name)
            time.sleep(0.5)  # Allow time for update
            return True
        except Exception as e:
            self.get_logger().error(f"Failed to remove collision object: {e}")
            return False


def main(args=None):
    """Main function for testing."""
    rclpy.init(args=args)

    try:
        controller = PickPlaceController()

        # Test: Move to home
        controller.move_to_home()

        rclpy.spin(controller)

    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
