#!/usr/bin/env python3
"""
Isaac ROS Debug Configuration Tool

Generates launch and configuration files for debugging VSLAM issues.
Init Sets maximum logging and diagnostic output for troubleshooting.
"""

import yaml
import argparse
import sys


def create_debug_launch_file():
    """Create debug launch configuration for maximal visibility"""
    debug_config = {
        "launch": {
            "args": {
                "use_sim_time": "true",
                "enable_debug": "true",
                "enable_benchmark": "true",
                "enable_visualization": "true"
            },
            "nodes": {
                "visual_slam": {
                    "package": "isaac_ros_visual_slam",
                    "executable": "visual_slam_node",
                    "name": "visual_slam_debug",
                    "output": "screen",
                    "parameters": [{
                        "enable_localization_n_mapping": True,
                        "enable_slam_visualization": True,
                        "enable_imu_fusion": True,
                        "enable_debug_logging": True,
                        "debug_print_rate": 10,
                        "enable_profiling": True,
                        "log_level": "DEBUG",
                        "benchmark_enable": True,
                        "benchmark_results_log": True,
                        "enable_detailed_metrics": True
                    }]
                }
            }
        }
    }

    return debug_config


def create_debug_rviz_config():
    """Create RViz configuration for VSLAM debugging"""
    rviz_config = """Panels:
  - Class: rviz_common/Displays
    Name: Displays
  - Class: rviz_common/Views
    Name: Views

Visualization Manager:
  Class: ""
  Displays:
    - Class: rviz_default_plugins/Image
      Name: Left Camera
      Topic: /stereo_camera/left/image_raw
    - Class: rviz_default_plugins/Image
      Name: Right Camera
      Topic: /stereo_camera/right/image_raw
    - Class: rviz_default_plugins/Map
      Name: Occupancy Grid
      Topic: /map
    - Class: rviz_default_plugins/Path
      Name: SLAM Path
      Topic: /visual_slam/tracking/vo_path
    - Class: rviz_default_plugins/MarkerArray
      Name: SLAM Features
      Topic: /visual_slam/features
    - Class: rviz_default_plugins/TF
      Name: TF Transforms
      Show Arrows: true

    # Add detailed debugging frames
    - Class: rviz_default_plugins/Grid
      Name: Grid
      Frame: base_link
      Plane: XY
      Color: 100; 100; 100
    - Class: rviz_default_plugins/Odometry
      Name: SLAM Odometry
      Topic: /visual_slam/tracking/odometry
      Arrow Length: 0.3
      Axes Length: 0.3

Tools:
  - Class: rviz_default_plugins/Interact
    Hide Inactive Objects: true
  - Class: rviz_default_plugins/Measure
    Line color: 128; 255; 128

Transformation:
  Head Pose Frame: base_link
  Reference Frame: map

Window Geometry:
  Width: 1280
  Height: 720
"""

    return rviz_config


def create_system_profiler():
    """Create script to profile system resources during VSLAM"""
    profiler_script = """#!/bin/bash
# Isaac ROS System Profiler

echo "=== Isaac ROS System Profiler ==="
echo "Starting resource monitoring for VSLAM debugging..."

# Monitor GPU stats
watch -n 1 -d '
  echo "=== GPU Status ===";
  nvidia-smi --format=csv,noheader,nounits --query-gpu=name,memory.used,memory.total,utilization.gpu;
  echo;
  echo "=== CPU Usage ===";
  top -bn1 | grep "Cpu(s)" | sed "s/.*, *\\([0-9.]*\\)%* id.*/\\1/" | awk '{print "CPU Load: " 100 - $1"%"}';
  echo;
  echo "=== ROS2 Nodes ===";
  ros2 node list | head -5;'

# Log to file
timestamp=$(date +%Y%m%d_%H%M%S)
mkdir -p /tmp/isaac_debug_logs
nvidia-smi -l 1 -f /tmp/isaac_debug_logs/gpu_${timestamp}.log &
top -b -d 1 -p $(pgrep -d',' ros2) > /tmp/isaac_debug_logs/ros2_${timestamp}.log &

echo "Logs saved to /tmp/isaac_debug_logs/"
echo "To stop profiling, press Ctrl+C"
"""

    return profiler_script


def create_imu_checker():
    """Template for checking IMU calibration alignment"""
    imu_config = {
        "imu_checker_node": {
            "ros__parameters": {
                "expected_gxyz": [0.1, 0.1, 9.71],  # Earth's gravity
                "expected_gxyz_stdev": [0.1, 0.1, 0.1],
                "expected_angular_velocity": [0.0, 0.0, 0.0],
                "expected_accel_drift": [0.1, 0.1, 0.1],
                "gyro_stdev_threshold": 0.25,
                "accel_stdev_threshold": 0.25,
                "angular_rotation": [0.8, 0.8, 0.8],
                "rotation_sigma”: 0.1
            }
        }
    }

    return imu_config


def main():
    parser = argparse.ArgumentParser(description='Generate Isaac ROS debug configurations')
    parser.add_argument('--type', choices=['launch', 'rviz', 'profiler', 'imu'],
                       default='launch', help='Type of debug file to generate')
    parser.add_argument('--output', help='Output file path')
    parser.add_argument('--verbose', '-v', action='store_true', help='Enable verbose output')

    args = parser.parse_args()

    if args.verbose:
        print(f"Generating debug configuration: {args.type}")

    if args.type == 'launch':
        config = create_debug_launch_file()
        output_file = args.output or "debug_isaac.launch.py"

        with open(output_file, 'w') as f:
            f.write("#!/usr/bin/env python3\n")
            f.write("# Debug launch configuration for Isaac ROS tests\n\n")
            f.write(f"debug_config = {repr(config)}\n")
            f.write("\n# Use with: ros2 launch <package> debug_isaac.launch.py\n")

    elif args.type == 'rviz':
        config = create_debug_rviz_config()
        output_file = args.output or "debug_isaac.rviz"

        with open(output_file, 'w') as f:
            f.write(config)

    elif args.type == 'profiler':
        config = create_system_profiler()
        output_file = args.output or "profile_isaac.sh"

        with open(output_file, 'w') as f:
            f.write(config)

    elif args.type == 'imu':
        config = create_imu_checker()
        output_file = args.output or "imu_checker_config.yaml"

        import yaml
        with open(output_file, 'w') as f:
            yaml.dump(config, f, default_flow_style=False)

    print(f"Debug configuration saved to: {output_file}")
    print("\nUsage examples:")
    print("  ros2 launch <package> debug_isaac.launch.py")
    print("  rviz2 -d debug_isaac.rviz")
    print("  ./profile_isaac.sh")


if __name__ == '__main__':
    main()