# Quick Start Guide - Minimal Working Demo

## ✅ What's Ready

You now have a **minimal working pick-and-place demo** that you can test immediately!

### Files Created:
1. ✅ `pick_place_controller.py` - MoveIt motion planning controller
2. ✅ `gripper_controller.py` - Simulated gripper with object attachment
3. ✅ `simple_pick_place_demo.py` - Complete demo script
4. ✅ `pick_place_params.yaml` - Configuration file
5. ✅ `README.md` - Full documentation
6. ✅ Package files (package.xml, setup.py, etc.)

## 🚀 How to Run the Demo

### Step 1: Build the Package

```bash
cd /Users/yaguan/Code/ar4
source /opt/ros/humble/setup.bash
colcon build --packages-select ar4_pick_and_place_examples
source install/setup.bash
```

### Step 2: Launch Isaac Sim with AR4

In **Terminal 1**:
```bash
cd /Users/yaguan/Code/ar4
./docker/run.sh -s ar4_isaac --build

# Once inside the container:
source /opt/ros/humble/setup.bash
colcon build --packages-select ar4_pick_and_place_examples
source install/setup.bash
ros2 launch ar4_isaac moveit.launch.py
```

### Step 3: (Optional) Spawn Objects in Isaac Sim

For testing, you'll need objects at the pick and place locations. You can:

**Option A: Manually add objects in Isaac Sim GUI**
1. Open Isaac Sim interface
2. Create → Shapes → Cube
3. Position at (0.3, 0.1, 0.025) - this is the pick location
4. Create another cube at (0.3, -0.15, 0.025) as reference

**Option B: Create objects programmatically** (we'll implement this later)

### Step 4: Run the Demo

In **Terminal 2** (inside the docker container or with sourced workspace):
```bash
ros2 run ar4_pick_and_place_examples simple_pick_place_demo
```

## 📍 Default Object Positions

The demo uses these hardcoded positions (configurable via ROS parameters):

- **Pick Location**: (0.3, 0.1, 0.025) meters - Red cube
- **Place Location**: (0.3, -0.15, 0.025) meters - Where to place the object
- **Object Size**: 5cm x 5cm x 5cm cube

## 🎛️ Customizing Positions

You can override positions using ROS parameters:

```bash
ros2 run ar4_pick_and_place_examples simple_pick_place_demo --ros-args \
  -p pick_x:=0.35 \
  -p pick_y:=0.15 \
  -p pick_z:=0.025 \
  -p place_x:=0.35 \
  -p place_y:=-0.20 \
  -p place_z:=0.025
```

## 🔍 What the Demo Does

The demo executes an 11-step pick-and-place sequence:

1. **Move to home** - All joints at 0°
2. **Open gripper** - Prepare for grasping
3. **Move to approach** - Position above object (10cm clearance)
4. **Move down to grasp** - Lower to object height
5. **Close gripper** - Simulate grasping
6. **Attach object** - Add to MoveIt planning scene (simulated)
7. **Lift object** - Raise 10cm
8. **Move to place approach** - Position above place location
9. **Move down to place** - Lower to place height
10. **Release object** - Detach from planning scene and open gripper
11. **Return home** - Back to starting position

Each step includes:
- ✅ Planning verification
- ✅ Collision checking
- ✅ Progress logging
- ✅ Error handling

## 📊 Expected Output

You should see output like this:

```
============================================================
Simple Pick-and-Place Demo Starting
============================================================
Pick position: (0.300, 0.100, 0.025)
Place position: (0.300, -0.150, 0.025)
Object size: [0.05, 0.05, 0.05]
Gripper mode: simulated
============================================================

============================================================
STEP 1: Moving to home position
============================================================
[INFO] Planning to joint target: [0.000, 0.000, 0.000, 0.000, 0.000, 0.000]
[INFO] Successfully reached joint target

...

============================================================
✓ DEMO COMPLETED SUCCESSFULLY!
============================================================
```

## ⚠️ Troubleshooting

### "Failed to initialize MoveIt"
- **Cause**: MoveIt nodes not running
- **Solution**: Ensure `ros2 launch ar4_isaac moveit.launch.py` is running in Terminal 1

### "Failed to move to [position]"
- **Cause**: MoveIt couldn't find valid motion plan
- **Solution**: 
  - Check that target positions are reachable
  - Increase `planning_time` in config/pick_place_params.yaml
  - Verify no collision objects blocking path

### "Planning took too long"
- **Cause**: Complex environment or tight constraints
- **Solution**: Increase velocity/acceleration scaling in config (try 0.2 instead of 0.1)

### Build Errors
- **Cause**: Missing dependencies
- **Solution**: 
  ```bash
  sudo apt install ros-humble-moveit ros-humble-moveit-py
  pip3 install numpy transforms3d
  ```

## 🎯 Next Steps

After successfully running the minimal demo, you can:

1. **Test with different positions** - Modify parameters to explore workspace
2. **Add object spawning** - We can implement automatic object creation in Isaac Sim
3. **Implement vision system** - Add camera and object detection
4. **Create launch files** - Single command to start everything
5. **Add more complex trajectories** - Cartesian paths, obstacle avoidance, etc.

## 💡 Tips

- **Start simple**: Use the default positions first
- **Watch in RViz**: MoveIt's RViz window shows planned trajectories
- **Check Isaac Sim**: Verify object positions visually
- **Read logs**: Detailed step-by-step information in terminal
- **Adjust speed**: Modify velocity_scaling for faster/slower motion

## 📚 More Information

See the full README.md for:
- Complete API reference
- Advanced configuration options
- Vision-based pick-and-place (coming next)
- Troubleshooting guide
- Architecture details

## ✅ Success Criteria

Your demo is working correctly if:
- ✅ Robot moves through all 11 steps without errors
- ✅ No planning failures or timeouts
- ✅ Object attaches/detaches in MoveIt planning scene
- ✅ Robot returns to home position
- ✅ Terminal shows "DEMO COMPLETED SUCCESSFULLY!"

---

**Ready to test?** Run the build commands above and start the demo!

**Questions or issues?** Check the troubleshooting section or the main README.md.
