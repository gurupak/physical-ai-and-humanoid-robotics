# Chapter 3 Quick Start Guide: NVIDIA Isaac AI Brain

:::tip[Time Needed]
15 minutes for basic setup | 30 minutes for full demo
:::

## Prerequisites
- RTX 3060+ GPU with 8GB+ VRAM
- Ubuntu 20.04/22.04
- ROS2 Humble installed
- 20GB free disk space

## Step 1: Install Isaac Sim (5 minutes)

```bash
# Download Isaac Sim 4.0
wget https://developer.nvidia.com/downloads/isaac-sim-4-0-0.tar.xz

# Extract
tar -xf isaac-sim-4-0-0.tar.xz
cd isaac-sim-4.0.0

# Run installer
./isaac-sim.sh --install-deps
```

## Step 2: Quick Launch (2 minutes)

```bash
# Launch Isaac Sim
./isaac-sim.sh --no-window  # Headless mode for quick test

# Verify installation
curl -k http://localhost:8211/status
```

Expected output: `{"status": "ready"}`

## Step 3: ROS2 Integration (3 minutes)

```bash
# Install Isaac ROS
sudo apt update
sudo apt install ros-humble-isaac-ros-visual-slam

# Source ROS
source /opt/ros/humble/setup.bash

# Test VSLAM node
ros2 run isaac_ros_visual_slam visual_slam_node --help
```

## Step 4: Run Demo (10 minutes)

### In Terminal 1:
```bash
# Launch pre-configured demo
cd isaac-sim-4.0.0
./python.sh standalone_examples/demo/humanoid_navigation.py
```

### In Terminal 2:
```bash
# Monitor VSLAM
rviz2 -d `ros2 pkg prefix isaac_ros_visual_slam`/share/isaac_ros_visual_slam/config/default.rviz
```

### In Terminal 3:
```bash
# Check topics
ros2 topic list | grep slam

# Expected output:
# /visual_slam/status
# /visual_slam/tracking/odometry
# /visual_slam/tracking/vo_path
```

## Quick Performance Check

Run this command to verify 30+ FPS:
```bash
ros2 topic hz /visual_slam/tracking/odometry
```

Expected: `average rate: 30.0-45.0 Hz`

## Common Quick Issues

| Issue | Quick Fix |
|-------|-----------|
| "CUDA error" | Check `nvidia-smi` output |
| "Permission denied" | Run `sudo chmod +x ./isaac-sim.sh` |
| "ROS2 not found" | Ensure `source /opt/ros/humble/setup.bash` |
| "Low FPS" | Close other GPU applications |

## Next Steps

1. Run full synthetic data generation tutorial
2. Configure custom humanoid robot
3. Integrate with Nav2 for navigation

## Quick Verification Script

Create `verify_setup.sh`:
```bash
#!/bin/bash
echo "=== Isaac Sim Quick Check ==="
echo "GPU: $(nvidia-smi --query-gpu=name --format=csv,noheader)"
echo "CUDA: $(nvcc --version | grep release)"
echo "Isaac: $(curl -s -k http://localhost:8211/status || echo 'Not running')"
echo "ROS: $(ros2 --version)"
echo "Topics: $(ros2 topic list | wc -l) total topics"
echo "==========================="
```

Run: `bash verify_setup.sh`

---

Ready? Continue to [full Chapter 3 tutorial](./chapter-3-isaac-ai-brain.md)