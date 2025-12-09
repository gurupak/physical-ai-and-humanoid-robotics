# Common Errors - Chapter 3: The AI-Robot Brain

This section documents frequently encountered issues and their solutions when working with NVIDIA Isaac platform.

## Isaac Sim Installation Issues

### "RTX GPU Not Detected" Error

**Symptoms**: Isaac Sim fails to start with "RTX-capable GPU required" message

**Root Cause**: No RTX GPU or outdated drivers

**Solution**:
1. Verify GPU model: `nvidia-smi --query-gpu=name --format=csv,noheader`
2. Minimum requirement: RTX 3060 (8GB VRAM)
3. Update drivers: `sudo apt install nvidia-driver-535`
4. Restart after driver update

### CUDA Version Mismatch

**Symptoms**: "CUDA 11.8+ required" error during installation

**Root Cause**: CUDA version incompatible with Isaac Sim

**Solution**:
```bash
# Check current CUDA version
nvcc --version

# Install compatible CUDA
wget https://developer.nvidia.com/compute/cuda/11.8.0/local_installers/cuda_11.8.0_520.61.05_linux.run
sudo sh cuda_11.8.0_520.61.05_linux.run
```

### Insufficient VRAM

**Symptoms**: Scenes fail to load or render at low quality

**Root Cause**: GPU memory exceeds available VRAM

**Solution**:
1. Reduce simulation complexity:
   - Lower texture quality in preferences
   - Reduce scene polygon count
   - Disable complex lighting
2. Monitor VRAM: `nvidia-smi -l 1`
3. **Minimum specs**: 8GB VRAM, **Recommended**: 16GB+ VRAM

## VSLAM Performance Issues

### Low Frame Rate (Below 30 FPS)

**Symptoms**: VSLAM drops below real-time performance

**Diagnostic Steps**:
```bash
# Check FPS
topic hz /visual_slam/tracking/odometry

# Monitor system resources
htop
```

**Solutions**:
1. **Reduce features**: Set max_features to 800
2. **Lower resolution**: Use 640x480 instead of 1920x1080
3. **GPU optimization**: Ensure GPU acceleration enabled
4. **Background processes**: Close other GPU applications

### Tracking Loss in Dynamic Scenes

**Symptoms**: "Lost tracking" warnings, position jumps

**Root Cause**: Insufficient static features in dynamic environments

**Solutions**:
1. **Increase feature density**
2. **Add static markers** to environment
3. **Reduce camera motion** during initialization
4. **Improve lighting** for better feature detection

### Feature Detection Failures

**Symptoms**: "Warning: Too few features detected"

**Diagnostic**:
```bash
ros2 topic echo /visual_slam/status
# Check feature count in output
```

**Solutions**:
1. **Lower feature quality threshold**: `feature_quality_threshold: 0.01`
2. **Increase minimum feature distance**: Not too close
3. **Use better texturing**: Add more visible edges
4. **Verify camera calibration** is current

## Navigation Configuration Problems

### Step Timing Issues

**Symptoms**: Robot stops between steps or moves erratically

**Root Cause**: Step duration too short/long for robot dynamics

**Solution**:
```yaml
# In Nav2 configuration
controller_server:
  ros__parameters:
    step_duration: 0.8  # seconds
    double_support_ratio: 0.2  # % of cycle in double support
```

### Collision Detection with Bipedal Motion

**Symptoms**: Robot starts/stops frequently, suboptimal paths

**Root Cause**: Costmap doesn't account for foot movement

**Solution**:
```yaml
# In costmap configuration
plugins: ["static_layer", "obstacle_layer", "footstep_layer"]
footstep_layer:
  plugin: "nav2_footstep_costmap_plugin/FootstepLayer"
  foot_clearance: 0.05  # 5cm clearance minimum
  step_size: [0.4, 0.3]  # [length, width]
```

### ZMP Stability Violations

**Symptoms**: Simulation shows robot becoming unstable

**Root Cause**: Zero Moment Point exceeds support polygon

**Diagnostic Commands**:
```bash
rostopic echo /humanoid_controller/zmp_status
# Check for stability warnings
```

**Solutions**:
1. **Reduce step length** to maintain stability
2. **Lower CoM height** in robot model
3. **Implement reactive stepping** controller
4. **Add stability feedback loop**

## Performance and Resource Issues

### High CPU Usage

**Symptoms**: System becomes unresponsive

**Diagnostic**:
```bash
# Check resource usage
top -H # Thread view
nvidia-smi # GPU memory
```

**Solutions**:
1. **Limit physics simulation frequency**
2. **Reduce render resolution**
3. **Disable unnecessary plugins**
4. **Use simplified robot models**

### Memory Leaks in Long Simulations

**Symptoms**: Gradual increase in memory usage over time

**Mitigation**:
1. **Restart simulation** periodically
2. **Use memory profiling tools**
3. **Clear buffers** between runs
4. **Update to latest versions**

## Integration and Workflow Issues

### Topic Mismatches

**Symptoms**: Messages not being received across components

**Troubleshooting**:
```bash
# Check topic list
ros2 topic list | grep -E "(stereo|vslam|nav2)"

# Verify message types match
ros2 topic info /stereo_camera/left/image_raw
```

**Solutions**:
1. **Confirm topic names** match launch files
2. **Check message type compatibility**
3. **Verify QoS settings** align between nodes
4. **Use correct TF transforms** linking coordinate frames

### Render Issues in Headless Mode

**Symptoms**: Screenshots all black in headless Ubuntu server

**Solution**:
```bash
# Use software rendering fallback
export MESA_GL_VERSION_OVERRIDE=3.3
export LIBGL_ALWAYS_SOFTWARE=1

# Or ensure X11 forwarding enabled
export DISPLAY=:0
```

### Doxusaurus Build Errors

**Symptoms**: MDX files fail to build with Isaac-specific content

**Common Issues**:
1. **Mermaid syntax errors**: < /> syntax differences
2. **MDX component compatibility**
3. **Image path references**

**Solutions**:
```bash
# Debug MDX issues
npm run build:dev  # Shows detailed errors

# Common fixes:
# 1. Check mermaid version compatibility
# 2. Use relative paths for images
# 3. Verify JSX expressions in mermaid
```

## Where to Get Help

1. **Check logs first**: `/var/log/nvidia-isaac-sim.log`
2. **NVIDIA forums**: [Isaac Sim Forum](https://forums.developer.nvidia.com/c/agx-autonomous-machines/isaac/45)
3. **GitHub issues**: Report reproducible bugs
4. **Community Discord**: Join robotics channels

---

**Issue resolved?** Continue exploring [Chapter 3 content](./index.md)

**New problem?** Report it with detailed system information and terminal output."}''