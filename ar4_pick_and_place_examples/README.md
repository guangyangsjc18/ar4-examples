# AR4 Pick-and-Place Examples

Comprehensive vision-based pick-and-place examples for the AR4 robotic arm using MoveIt 2 and Isaac Sim.

## Overview

This package provides complete, ready-to-use examples demonstrating:

- **Vision-based object detection** using depth cameras
- **MoveIt 2 motion planning** for collision-free trajectories
- **Pick-and-place operations** with simulated grasping
- **Automatic object spawning** in Isaac Sim
- **Relative placement** (placing objects next to detected references)

## Features

### Core Capabilities

✅ **Two Operation Modes:**
- Simple pick-and-place with hardcoded positions
- Vision-based pick-and-place with automatic object detection

✅ **Gripper Support:**
- Simulated attachment (using MoveIt's AttachedCollisionObject)
- Physical gripper control (ready for hardware integration)

✅ **Robust Perception:**
- Depth-based object segmentation
- Point cloud processing
- TF frame management

✅ **Isaac Sim Integration:**
- Programmatic object spawning
- Camera setup and configuration
- USD model support

## Prerequisites

### System Requirements

- ROS 2 Humble
- MoveIt 2
- Isaac Sim (with ROS 2 bridge)
- Python 3.8+

### Dependencies

**ROS 2 Packages:**
```bash
sudo apt install ros-humble-moveit ros-humble-moveit-py \
  ros-humble-cv-bridge ros-humble-vision-msgs \
  ros-humble-tf2-ros ros-humble-tf2-geometry-msgs
```

**Python Packages:**
```bash
pip3 install numpy opencv-python transforms3d scipy
```

## Installation

1. **Navigate to your workspace:**
```bash
cd ~/ws/src  # Or your AR4 workspace
```

2. **The package should already be in the `ar4` repository**

3. **Build the workspace:**
```bash
cd ~/ws
source /opt/ros/humble/setup.bash
colcon build --packages-select ar4_pick_and_place_examples
source install/setup.bash
```

## Quick Start

### 1. Launch Isaac Sim with AR4

```bash
# Terminal 1: Start Isaac Sim with MoveIt
ros2 launch ar4_isaac moveit.launch.py
```

### 2. Add Camera to Scene (First Time Setup)

See [Camera Setup Guide](#camera-setup) below for detailed instructions.

### 3. Run Simple Pick-and-Place Demo

```bash
# Terminal 2: Spawn demo objects
ros2 run ar4_pick_and_place_examples spawn_demo_objects

# Terminal 3: Run simple demo
ros2 run ar4_pick_and_place_examples simple_pick_place_demo
```

### 4. Run Vision-Based Demo

```bash
# Terminal 2: Run vision-based demo
ros2 run ar4_pick_and_place_examples vision_pick_place_demo
```

## Package Structure

```
ar4_pick_and_place_examples/
├── ar4_pick_and_place_examples/     # Python package
│   ├── pick_place_controller.py    # Main controller (MoveIt interface)
│   ├── gripper_controller.py       # Gripper abstraction layer
│   ├── object_spawner.py            # Isaac Sim object spawning
│   ├── vision_processor.py          # Vision/perception module
│   ├── simple_pick_place_demo.py    # Demo 1: Hardcoded positions
│   ├── vision_pick_place_demo.py    # Demo 2: Vision-based
│   ├── spawn_demo_objects.py        # Object spawning script
│   ├── test_gripper.py              # Gripper testing utility
│   └── utils/                       # Utility modules
│       ├── transforms.py            # TF2 helpers
│       └── trajectory_utils.py      # Trajectory generation
├── config/                           # Configuration files
│   ├── pick_place_params.yaml      # Main parameters
│   ├── camera_config.yaml          # Vision parameters
│   └── object_definitions.yaml     # Object spawn config
├── launch/                           # Launch files
│   ├── demo_simple_pick_place.launch.py
│   ├── demo_vision_pick_place.launch.py
│   └── spawn_objects.launch.py
├── README.md                         # This file
└── package.xml                       # Package manifest
```

## Camera Setup

### Adding a Camera to Isaac Sim

The AR4 needs a camera for vision-based operations. Here's how to add one:

#### Method 1: Add Camera to URDF (Recommended)

Create `ar4_isaac/urdf/ar4_with_camera.urdf.xacro`:

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="ar4_with_camera">
  
  <!-- Include base AR4 -->
  <xacro:include filename="$(find ar4_description)/urdf/ar4.urdf.xacro" />
  
  <!-- Camera Link -->
  <link name="camera_link">
    <visual>
      <geometry>
        <box size="0.02 0.08 0.02"/>
      </geometry>
      <material name="camera_blue">
        <color rgba="0.0 0.0 0.8 1.0"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.02 0.08 0.02"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.1"/>
      <inertia ixx="0.0001" ixy="0.0" ixz="0.0" 
               iyy="0.0001" iyz="0.0" izz="0.0001"/>
    </inertial>
  </link>
  
  <!-- Camera Joint (mounted above workspace) -->
  <joint name="camera_joint" type="fixed">
    <parent link="base_link"/>
    <child link="camera_link"/>
    <origin xyz="0.0 0.0 0.6" rpy="0 ${pi/4} 0"/>
  </joint>
  
  <!-- Camera Optical Frame -->
  <link name="camera_optical_link"/>
  
  <joint name="camera_optical_joint" type="fixed">
    <parent link="camera_link"/>
    <child link="camera_optical_link"/>
    <origin xyz="0 0 0" rpy="${-pi/2} 0 ${-pi/2}"/>
  </joint>

</robot>
```

#### Method 2: Spawn Camera Programmatically in Isaac Sim

Add to `run_sim.py`:

```python
from omni.isaac.sensor import Camera

# In spawn_ar4 or create_world function
camera = Camera(
    prim_path="/World/camera",
    position=[0.0, 0.0, 0.6],
    orientation=[0.383, 0.0, 0.0, 0.924],  # Looking down at ~45 degrees
)
camera.initialize()
camera.set_focal_length(24.0)
camera.set_resolution((640, 480))
```

#### Method 3: Manual Scene Setup (Quickest for Testing)

1. Open Isaac Sim
2. In the menu: **Create → Camera**
3. Position it at (0, 0, 0.6) looking down at the workspace
4. Enable ROS 2 bridge: **Isaac Utils → ROS2 Camera Helper**

## Configuration

### Pick-Place Parameters (`config/pick_place_params.yaml`)

```yaml
pick_place:
  ros__parameters:
    # MoveIt settings
    planning_group: "arm"
    end_effector_link: "link_6"
    base_frame: "base_link"
    planning_time: 10.0
    velocity_scaling: 0.1        # 10% of max speed
    acceleration_scaling: 0.1
    
    # Pick/place behavior
    approach_distance: 0.10       # meters above object
    retreat_distance: 0.10        # meters to lift after grasp
    grasp_offset_z: 0.0           # z-offset from detected pose
    place_offset_x: 0.15          # meters to the side of reference
    
    # Gripper
    gripper_mode: "simulated"     # or "physical"
    
    # Safety
    max_planning_attempts: 5
    planning_timeout: 5.0
```

### Vision Parameters (`config/camera_config.yaml`)

```yaml
vision:
  ros__parameters:
    camera_frame: "camera_optical_link"
    rgb_topic: "/camera/rgb/image_raw"
    depth_topic: "/camera/depth/image_raw"
    camera_info_topic: "/camera/rgb/camera_info"
    
    # Detection
    detection_method: "depth_clustering"  # or "color"
    
    # Depth clustering parameters
    depth_threshold: 0.02         # meters
    min_cluster_size: 100         # points
    max_cluster_size: 10000
    
    # Color detection (if using color method)
    color_lower_hsv: [0, 100, 100]
    color_upper_hsv: [10, 255, 255]
    
    # Filtering
    workspace_min: [0.1, -0.3, 0.0]   # [x, y, z] in meters
    workspace_max: [0.5, 0.3, 0.3]
```

### Object Definitions (`config/object_definitions.yaml`)

```yaml
objects:
  pick_object:
    type: "cube"
    size: [0.05, 0.05, 0.05]
    color: [1.0, 0.0, 0.0]       # Red
    mass: 0.1
    pose:
      position: [0.3, 0.1, 0.025]
      orientation: [0, 0, 0, 1]
  
  reference_object:
    type: "cube"
    size: [0.05, 0.05, 0.05]
    color: [0.0, 0.0, 1.0]       # Blue
    mass: 0.1
    pose:
      position: [0.3, -0.15, 0.025]
      orientation: [0, 0, 0, 1]
  
  table:
    type: "box"
    size: [0.8, 0.6, 0.02]
    color: [0.7, 0.7, 0.7]       # Gray
    mass: 10.0
    pose:
      position: [0.3, 0.0, -0.01]
      orientation: [0, 0, 0, 1]
```

## Usage Examples

### Example 1: Simple Pick-and-Place

Hardcoded positions, no vision required:

```python
import rclpy
from ar4_pick_and_place_examples.pick_place_controller import PickPlaceController
from ar4_pick_and_place_examples.gripper_controller import GripperController
from geometry_msgs.msg import Pose

rclpy.init()

# Initialize controllers
pick_place = PickPlaceController()
gripper = GripperController(mode='simulated')

# Define poses
pick_pose = Pose()
pick_pose.position.x = 0.3
pick_pose.position.y = 0.1
pick_pose.position.z = 0.025
pick_pose.orientation.w = 1.0

place_pose = Pose()
place_pose.position.x = 0.3
place_pose.position.y = -0.15
place_pose.position.z = 0.025
place_pose.orientation.w = 1.0

# Execute pick-and-place
# 1. Move to home
pick_place.move_to_home()

# 2. Move to approach pose (above object)
approach_pose = pick_place.compute_approach_pose(pick_pose)
pick_place.move_to_pose(approach_pose)

# 3. Open gripper
gripper.open_gripper()

# 4. Move down to grasp
pick_place.move_to_pose(pick_pose)

# 5. Close gripper / attach object
gripper.close_gripper()
gripper.attach_object("pick_object")

# 6. Lift object
retreat_pose = pick_place.compute_retreat_pose(pick_pose)
pick_place.move_to_pose(retreat_pose)

# 7. Move to place approach
place_approach = pick_place.compute_approach_pose(place_pose)
pick_place.move_to_pose(place_approach)

# 8. Move down to place
pick_place.move_to_pose(place_pose)

# 9. Release object
gripper.open_gripper()
gripper.detach_object("pick_object")

# 10. Retreat and return home
place_retreat = pick_place.compute_retreat_pose(place_pose)
pick_place.move_to_pose(place_retreat)
pick_place.move_to_home()

rclpy.shutdown()
```

### Example 2: Vision-Based Pick-and-Place

Automatic object detection:

```python
import rclpy
from ar4_pick_and_place_examples.vision_processor import VisionProcessor
from ar4_pick_and_place_examples.pick_place_controller import PickPlaceController
from ar4_pick_and_place_examples.gripper_controller import GripperController

rclpy.init()

# Initialize
vision = VisionProcessor()
pick_place = PickPlaceController()
gripper = GripperController(mode='simulated')

# Wait for vision data
while not vision.has_data():
    rclpy.spin_once(vision, timeout_sec=0.1)

# Detect objects
objects = vision.detect_objects()
if len(objects) < 2:
    print("Need at least 2 objects!")
    exit(1)

# Sort by color (red objects first)
pick_object = objects[0]   # First detected object
reference_object = objects[1]  # Second detected object

# Compute place pose (to the side of reference)
place_pose = pick_place.compute_relative_place_pose(
    reference_pose=reference_object.pose,
    offset_x=0.15  # 15cm to the side
)

# Execute pick-and-place (similar to Example 1 but with detected poses)
# ... (same sequence as above)

rclpy.shutdown()
```

## Demos

### Demo 1: Simple Pick-and-Place

**Description:** Basic demo with hardcoded positions

**Command:**
```bash
ros2 run ar4_pick_and_place_examples simple_pick_place_demo
```

**What it does:**
1. Moves to home position
2. Picks red cube at (0.3, 0.1, 0.025)
3. Places it at (0.3, -0.15, 0.025)
4. Returns to home

**Expected behavior:** Robot picks up the red cube and places it next to the blue cube

### Demo 2: Vision-Based Pick-and-Place

**Description:** Full autonomous pick-and-place using vision

**Command:**
```bash
ros2 run ar4_pick_and_place_examples vision_pick_place_demo
```

**What it does:**
1. Waits for camera data
2. Detects all objects in workspace
3. Identifies pick target (red cube)
4. Identifies reference object (blue cube)
5. Computes placement location (to the side of blue cube)
6. Executes pick-and-place
7. Returns to home

**Expected behavior:** Robot autonomously detects objects and moves the red cube next to the blue cube

## Troubleshooting

### Camera not publishing

**Problem:** No image data received

**Solutions:**
- Check Isaac Sim ROS 2 bridge is enabled
- Verify topics: `ros2 topic list | grep camera`
- Restart Isaac Sim and ensure camera is in scene
- Check camera frame in TF tree: `ros2 run tf2_tools view_frames`

### Planning failures

**Problem:** MoveIt cannot find valid plans

**Solutions:**
- Increase `planning_time` in config (try 15-20 seconds)
- Increase `approach_distance` (try 0.15m)
- Check for collision objects blocking path
- Verify joint limits in `joint_limits.yaml`
- Try different IK solver in `kinematics.yaml`

### Objects not detected

**Problem:** Vision system doesn't find objects

**Solutions:**
- Verify objects are in camera view (check in RViz)
- Adjust `workspace_min/max` bounds in config
- Tune `depth_threshold` for clustering
- Check object sizes meet `min_cluster_size`
- Ensure adequate lighting in Isaac Sim scene

### Gripper not working

**Problem:** Object doesn't attach/detach

**Solutions:**
- Verify `gripper_mode` is set to `"simulated"`
- Check object name matches between spawner and controller
- Ensure MoveIt planning scene is updated (add delays)
- For physical mode, verify gripper controller is running

### Isaac Sim crashes

**Problem:** Simulation hangs or crashes

**Solutions:**
- Reduce number of objects in scene
- Lower camera resolution
- Increase `physics_dt` in `run_sim.py`
- Check NVIDIA drivers are up to date
- Ensure sufficient GPU memory

## API Reference

### PickPlaceController

```python
controller = PickPlaceController()

# Motion commands
controller.move_to_home() -> bool
controller.move_to_joint_positions(joint_positions: List[float]) -> bool
controller.move_to_pose(pose: Pose) -> bool
controller.plan_cartesian_path(waypoints: List[Pose]) -> bool

# Pose computation
controller.compute_approach_pose(grasp_pose: Pose, distance: float) -> Pose
controller.compute_retreat_pose(current_pose: Pose, distance: float) -> Pose

# State queries
controller.get_current_pose() -> PoseStamped
controller.get_current_joint_values() -> List[float]

# Scene management
controller.add_collision_box(name: str, pose: Pose, size: List[float]) -> bool
controller.remove_collision_object(name: str) -> bool
```

### GripperController

```python
gripper = GripperController(mode='simulated')  # or 'physical'

# Gripper commands
gripper.open_gripper() -> bool
gripper.close_gripper() -> bool
gripper.attach_object(object_id: str, link_name: str) -> bool
gripper.detach_object(object_id: str) -> bool
gripper.is_object_attached() -> bool
```

### VisionProcessor

```python
vision = VisionProcessor()

# Detection
vision.detect_objects() -> List[DetectedObject]
vision.get_object_pose(object_id: int) -> Pose

# Data availability
vision.has_data() -> bool
vision.wait_for_data(timeout: float) -> bool
```

### ObjectSpawner

```python
spawner = ObjectSpawner()

# Spawning
spawner.spawn_cube(name: str, pose: Pose, size: List[float], color: List[float])
spawner.spawn_cylinder(name: str, pose: Pose, radius: float, height: float)
spawner.spawn_table(pose: Pose, size: List[float])
spawner.spawn_from_usd(name: str, usd_path: str, pose: Pose)

# Management
spawner.delete_object(name: str)
spawner.get_object_pose(name: str) -> Pose
```

## Advanced Topics

### Custom Object Detection

Implement custom detection algorithms by extending `VisionProcessor`:

```python
from ar4_pick_and_place_examples.vision_processor import VisionProcessor

class CustomVisionProcessor(VisionProcessor):
    def detect_objects(self):
        # Your custom detection algorithm
        # Example: ML-based object detection
        pass
```

### Physical Gripper Integration

To use a real gripper, implement the physical gripper interface in `gripper_controller.py`:

```python
def _send_gripper_command_physical(self, position: float) -> bool:
    # Publish to your gripper's action server
    # Example: FollowJointTrajectory for gripper joints
    pass
```

### Adding New Object Types

Extend `ObjectSpawner` to support new shapes:

```python
def spawn_custom_shape(self, name, pose, params):
    # Use Isaac Sim USD API to create custom geometry
    pass
```

## Contributing

Contributions are welcome! Please:

1. Fork the repository
2. Create a feature branch
3. Add tests for new functionality
4. Submit a pull request

## License

BSD-3-Clause License. See `LICENSE` file for details.

## Citation

If you use this package in your research, please cite:

```bibtex
@software{ar4_pick_and_place_examples,
  title = {AR4 Pick-and-Place Examples},
  author = {Your Name},
  year = {2026},
  url = {https://github.com/your-repo/ar4}
}
```

## Support

- **Issues:** https://github.com/your-repo/ar4/issues
- **Discussions:** https://github.com/your-repo/ar4/discussions
- **Documentation:** https://your-docs-site.com

## Acknowledgments

- AR4 robot design by Annin Robotics
- Built on MoveIt 2 motion planning framework
- Isaac Sim by NVIDIA

## Changelog

### Version 0.1.0 (2026-02-09)
- Initial release
- Simple pick-and-place demo
- Vision-based pick-and-place demo
- Object spawning utilities
- Simulated gripper support
- Comprehensive documentation
