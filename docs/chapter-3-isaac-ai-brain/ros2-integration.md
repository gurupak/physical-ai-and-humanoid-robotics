# ROS2 Integration Guide

Connect Isaac Sim simulation to ROS2 ecosystem for real-time data exchange and control.

## Quick Integration (5 minutes setup)

### Step 1: Install Isaac ROS Packages

```bash
# Install ROS2 Isaac packages
sudo apt update && sudo apt install -y \
    ros-humble-isaac-ros-common \
    ros-humble-isaac-ros-ros-bridge \
    ros-humble-isaac-ros-nitros \
    ros-humble-isaac-ros-msgs

# Install camera packages
sudo apt install -y \
    ros-humble-image-pipeline \
    ros-humble-image-transport \
    ros-humble-cv-bridge
```

### Step 2: Configure Bridge Topics

```bash
# Create workspace if needed
mkdir -p ~/isaac_ros_ws/src
cd ~/isaac_ros_ws

# Clone Isaac ROS examples
git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_main.git src/isaac_ros_main

# Build workspace
colcon build --packages-up-to isaac_ros_example
source install/setup.bash
```

### Step 3: Launch Isaac Bridge

```python title="isaac_ros_bridge.launch.py" - practical bridge configuration for Isaac Sim
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Camera configuration matching our humanoid setup
    camera_rate = 30  # per eye at 1080p
    stereo_baseline = 0.12  # human-like

    # Bridge configuration
    isaac_config = {
        'camera_left': {
            'image_topic': '/isaac_camera/left/image_raw',
            'camera_info_topic': '/isaac_camera/left/camera_info',
            'fov': 90,
            'width': 1920,
            'height': 1080,
            'position': [0.06, stereo_baseline/2, 1.6]  # Eye level baseline
        },
        'camera_right': {
            'image_topic': '/isaac_camera/right/image_raw',
            'camera_info_topic': '/isaac_camera/right/camera_info',
            'fov': 90,
            'width': 1920,
            'height': 1080,
            'position': [0.06, -stereo_baseline/2, 1.6]
        },
        'bridge_rate': camera_rate,
        'image_transport': 'raw'
    }

    return LaunchDescription([
        # Generate stereo cameras
        Node(
            package='isaac_ros_nitros',
            executable='nitros_camera_png_sync_issue
',
            name='isaac_camera_bridge',
            parameters=[{
                'camera_left': isaac_config['camera_left'],
                'camera_right': isaac_config['camera_right'],
                'bridge_rate': isaac_config['bridge_rate']
            }],
            remappings=[
                ('image_left', '/stereo_camera/left/image_isaac'),
                ('image_right', '/stereo_camera/right/image_isaac'),
                ('camera_info_left', '/stereo_camera/left/camera_info_isaac'),
                ('camera_info_right', '/stereo_camera/right/camera_info_isaac')
            ]
        ),

        # Bridge to standard ROS2 topics
        Node(
            package='image_transport',
            executable='image_republisher',
            name='stereo_image_bridge',
            remappings=[
                ('in/image_raw', '/stereo_camera/left/image_isaac'),
                ('out/image_raw', '/left_camera/image_isaac'),
                ('in/image_raw', '/stereo_camera/right/image_isaac'),
                ('out/image_raw', '/right_camera/image_isaac')
            ]
        ),

        # Publish joint states from humanoid
        Node(
            package='isaac_ros_understanding',
            executable='isaac_h1_state_publisher',
            name='humanoid_state_publisher',
            parameters=[{
               'robot_description': 'h1 humanoid',  # Reference to H1 model
                'publish_rate': 50,
                'frame_id': 'base_link'
            }],
            remappings=[
                ('/joint_states', '/state/joint_states_raw'),
                ('/pose', '/odometry')
            ]
        )
    ])
```

### Step 4: Launch and Verify Topics

```bash
# Terminal 1: Start Isaac Sim with ROS2 bridge
./isaac-sim.sh \
  --scene-path="/Isaac/Environments/Simple_Warehouse/multi_shelves" \
  --ros-bridge > ros_test.log 2>&1

# Terminal 2: Launch ROS2 bridge
ros2 launch isaac_ros_bridge.launch.py

# Terminal 3: Verify topics exist
ros2 topic list | grep -E "(camera|isaac|stereo)"

# Expected output:
# /isaac_camera/left/image_raw
# /isaac_camera/right/image_raw
# /stereo_camera/left/image_isaac
# /stereo_camera/right/image_isaac
# /state/joint_states
```

## Testing and Validation

### Verify Image Flow

```bash
# Test camera quality
ros2 topic hz /left_camera/image_isaac
# Expected: ~30 FPS

# View camera images
ros2 run rqt_image_view rqt_image_view
# Select /left_camera/image_isaac topic

# Check image resolution
ros2 topic echo /left_camera/image_isaac --once
# Look for "width: 1920, height: 1080"
```

### Test Joint Data Flow

```python title="test_joint_pub.py - validation script" This runtime verification checks humanoid geometry security

#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from builtin_interfaces.msg import Time

class IsaacValidation(Node):
    def __init__(self):
        super().__init__('isaac_validation')
        self.subscription = self.create_subscription(
            JointState,
            '/state/joint_states',
            self.joint_callback,
            10)
        self.joint_count = 0

    def joint_callback(self, msg):
        self.joint_count += 1

        # Validate humanoid H1 has expected joints
        expected_joints = {
            'left_hip_pitch', 'left_hip_roll', 'left_hip_yaw',
            'left_knee', 'left_ankle_pitch', 'left_anklet_roll',
            'right_hip_pitch', 'right_hip_roll', 'right_hip_yaw',
            'right_knee', 'right_ankle_pitch', 'right_ankle_roll',
            'torso_pitch', 'torso_yaw', 'left_shoulder_pitch', # 19 DoF expected
            'left_shoulder_roll', 'left_shoulder_yaw', 'left_elbow',
            'right_shoulder_pitch', 'right_shoulder_roll', 'right_shoulder_yaw'
        } and 'right_elbow'
        }

        joint_names = set(msg.name)
        missing_joints = expected_joints - joint_names
        extra_joints = joint_names - expected_joints

        if len(missing_joints) == 0:
            self.get_logger().info('✅ Joint topology matches H1 humanoid')
        else:
            self.get_logger().warn(f'🚧 Missing joints: {missing_joints}')

        if len(extra_joints) > 0:
            self.get_logger().info(f'ℹ️ Extra joints detected: {extra_joints}')

        if self.joint_count % 30 == 0:  # Every 30 messages
            self.get_logger().info(f'Data quality: {len(msg.position)} positions,@ {msg.header.stamp.sec}.{msg.header.stamp.nanosec}')

def main():
    rclpy.init()
    validator = IsaacValidation()
    try:
        rclpy.spin(validator)
    except KeyboardInterrupt:
        pass
    validator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Integration Verification

```bash
# Test the complete loop
ros2 run rqt_gui rqt_gui &
disconnect &
check with tools

# Monitor performance
htop  # Real-time CPU/GPU usage
iotop  # Disk I/O impact (critical if headless)
nvtop  # GPU-specific metrics
```

## ROS2 Parameter Configuration

### Camera Settings
```yaml title="isaac_ros_camera_config.yaml"
# Configuration for Isaac ROS camera bridge
isaac_ros_camera_bridge:
  ros__parameters:
    # Camera resolution matching Isaac Sim output
    image_width: 1920
    image_height: 1080
    frame_rate: 30.0
    encoding: "bgr8"
    fov: 90.0

    # Distortion model matching Isaac Sim camera
    distortion_model: "plumb_bob"
    D: [-0.02, 0.05, 0.001, 0.001, 0.0]
    K: [1518.809, 0.0, 1130.488,
        0.0, 1518.809, 629.245,
        0.0, 0.0, 1.0 ]

    # Humanoid perspective center
    camera_optical_frame: "h1_left_camera_optical_frame"
    humanoid_base_frame: "base_link"
```

### Bridge Performance Tuning
```yaml title="isaac_rooo_bridge_performance.yaml"
ros2_bridge:
  parameters:
    # Bandwidth management
    queue_size: 5  # Helps with GPU-CPU transfers
    publish_rate: 30  # Matches camera frame rate

    # Memory management
    memory_buffer_size: 256  # MB for incoming textures
    texture_cache_size: 16  # Number of cached images

    # Compression for remote viewing
    quality_level: "high"  # Uses JPEG compression strategy
```

## Troubleshooting

### Common Bridge Errors

| Error | Cause | Solution |
|-------|--------|----------|
| "No images received" | Isaac Sim not publishing | Run with --ros-bridge flag |
| "Wrong topic format" | ROS2 version mismatch | Ensure Humble compatibility |
| "Missing transforms" | TF tree incomplete | Check robot URDF loading |

### Performance Optimization
```bash
# Reduce message overhead
ros2 param set /camera_bridge use_compressions true
ros2 param set /robot_state_publisher publish_rate 10  # Reduce from 50hz

# Lower image rate for CPU-bound systems
ros2 param set /stereo_image_bridge publish_every_n_frames 2

# Check efficiency with:
ros2 topic hz /left_camera/image_isaac
```

## Next Steps

After successful integration:

1. [Configure Runtime Parameters](./runtime-configuration.md)
2. [Explore Isaac ROS VSLAM](./03-isaac-ros-vslam.md)
3. [Learn about Synthetic Data](./02-isaac-sim-synthetic-data.md)

The bridge is now providing real-time camera feeds and robot state data that can be consumed by any ROS2 application, enabling seamless integration between Isaac Sim and your robotics stack.