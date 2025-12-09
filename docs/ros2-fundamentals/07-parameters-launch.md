---
id: 07-parameters-launch
title: "Parameters & Launch Files"
sidebar_label: "Parameters & Launch"
sidebar_position: 7
sidebar_custom_props:
  difficulty: "Intermediate"
  readingTime: "20 minutes"
  handsOnTime: "45 minutes"
---

import Quiz from '@site/src/components/Quiz';
import Callout from '@site/src/components/Callout';

# Parameters & Launch Files

Your nodes are working, but what if you need to configure them without changing code? What if you want to start 10 nodes at once? This sub-chapter covers **parameters** (runtime configuration) and **launch files** (multi-node orchestration).

---

## Part 1: Parameters

### What Are Parameters?

**Parameters** are node configuration values that can be set at runtime without recompiling. Think of them as settings you can adjust:

- Camera node: exposure time, resolution, frame rate
- Navigation node: max speed, obstacle distance threshold
- Logger node: log level, output file path

**Benefits**:
- Change behavior without code changes
- Different configurations for different environments (sim vs. real robot)
- Runtime reconfiguration

---

## Declaring and Using Parameters

### Basic Parameter Usage

```python title="my_first_package/param_node.py" showLineNumbers {7-12,16}
import rclpy
from rclpy.node import Node

class ParameterNode(Node):
    def __init__(self):
        super().__init__('parameter_node')
        
        # Declare parameters with default values
        self.declare_parameter('max_speed', 1.0)
        self.declare_parameter('robot_name', 'my_robot')
        self.declare_parameter('enable_logging', True)
        
        # Read parameter values
        max_speed = self.get_parameter('max_speed').value
        robot_name = self.get_parameter('robot_name').value
        enable_logging = self.get_parameter('enable_logging').value
        
        self.get_logger().info(f'Max Speed: {max_speed}')
        self.get_logger().info(f'Robot Name: {robot_name}')
        self.get_logger().info(f'Logging: {enable_logging}')

def main(args=None):
    rclpy.init(args=args)
    node = ParameterNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Key Points**:
- **Lines 9-11**: Declare parameters with defaults
- **Lines 14-16**: Read parameter values
- Parameters have types: `int`, `float`, `str`, `bool`, `array`

---

## Setting Parameters

### Method 1: Command Line

```bash
ros2 run my_first_package param_node --ros-args -p max_speed:=2.5 -p robot_name:=alpha
```

**Output**:
```
[INFO] [parameter_node]: Max Speed: 2.5
[INFO] [parameter_node]: Robot Name: alpha
[INFO] [parameter_node]: Logging: True
```

### Method 2: YAML Configuration File

Create `config/params.yaml`:

```yaml
parameter_node:
  ros__parameters:
    max_speed: 3.0
    robot_name: 'beta'
    enable_logging: false
```

Run with parameter file:

```bash
ros2 run my_first_package param_node --ros-args --params-file config/params.yaml
```

### Method 3: Runtime with CLI

```bash
# Get current value
ros2 param get /parameter_node max_speed

# Set new value
ros2 param set /parameter_node max_speed 4.0

# List all parameters
ros2 param list /parameter_node
```

<Quiz
  question="What is the advantage of using parameters instead of hardcoding values in your code?"
  options={[
    "Parameters make code run faster",
    "Parameters allow runtime configuration without recompiling",
    "Parameters reduce memory usage",
    "Parameters are required by ROS 2"
  ]}
  correctAnswer={1}
  explanation="Parameters allow you to change node behavior (max speed, thresholds, file paths) without modifying and recompiling code. You can also use different parameter files for simulation vs. real robot."
  difficulty="easy"
/>

---

## Dynamic Reconfiguration

Parameters can be changed while the node is running:

```python
def __init__(self):
    super().__init__('parameter_node')
    
    self.declare_parameter('max_speed', 1.0)
    
    # Add callback for parameter changes
    self.add_on_set_parameters_callback(self.parameter_callback)

def parameter_callback(self, params):
    for param in params:
        if param.name == 'max_speed':
            self.get_logger().info(f'Max speed changed to: {param.value}')
    
    return SetParametersResult(successful=True)
```

Now when you run:

```bash
ros2 param set /parameter_node max_speed 5.0
```

The node logs: `Max speed changed to: 5.0`

---

## Part 2: Launch Files

### What Are Launch Files?

**Launch files** start multiple nodes, set parameters, remap topics, and configure complex systems—all from a single command.

**Use Cases**:
- Start robot driver + sensor nodes + navigation stack
- Configure different setups (simulation vs. hardware)
- Set parameters for all nodes from one place

---

## Creating a Python Launch File

ROS 2 uses **Python** for launch files (more powerful than XML).

```python title="launch/my_first_launch.py" showLineNumbers
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Node 1: Publisher
        Node(
            package='my_first_package',
            executable='publisher',
            name='my_publisher',
            output='screen',
            parameters=[{'timer_period': 0.5}]
        ),
        
        # Node 2: Subscriber
        Node(
            package='my_first_package',
            executable='subscriber',
            name='my_subscriber',
            output='screen'
        ),
        
        # Node 3: Parameter Node
        Node(
            package='my_first_package',
            executable='param_node',
            name='configured_param_node',
            output='screen',
            parameters=[
                {'max_speed': 2.0},
                {'robot_name': 'launch_robot'}
            ]
        )
    ])
```

**Breakdown**:
- **Lines 7-13**: Start publisher with parameter
- **Lines 16-21**: Start subscriber
- **Lines 24-32**: Start parameter node with custom config
- **output='screen'**: Print logs to console

---

## Running Launch Files

### Directory Structure

```
my_first_package/
├── my_first_package/
│   ├── publisher_node.py
│   ├── subscriber_node.py
│   └── param_node.py
├── launch/
│   └── my_first_launch.py
├── config/
│   └── params.yaml
├── package.xml
└── setup.py
```

### Update setup.py

```python title="setup.py" {4,14-17}
import os
from glob import glob
from setuptools import setup

package_name = 'my_first_package'

setup(
    name=package_name,
    # ... other fields ...
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Install launch files
        (os.path.join('share', package_name, 'launch'),
         glob('launch/*.py')),
    ],
    # ... entry points ...
)
```

### Build and Run

```bash
cd ~/ros2_ws
colcon build --packages-select my_first_package
source ~/ros2_ws/install/setup.bash

ros2 launch my_first_package my_first_launch.py
```

**Output**: All three nodes start simultaneously!

```
[INFO] [my_publisher]: Publisher node started
[INFO] [my_subscriber]: Subscriber node started
[INFO] [configured_param_node]: Max Speed: 2.0
[INFO] [configured_param_node]: Robot Name: launch_robot
[INFO] [my_publisher]: Publishing: "Hello World: 0"
[INFO] [my_subscriber]: I heard: "Hello World: 0"
...
```

<Quiz
  question="What is the primary advantage of using launch files?"
  options={[
    "Launch files make nodes run faster",
    "Launch files allow starting and configuring multiple nodes with a single command",
    "Launch files are required to run any ROS 2 node",
    "Launch files reduce code complexity"
  ]}
  correctAnswer={1}
  explanation="Launch files orchestrate complex systems by starting multiple nodes, setting parameters, remapping topics, and configuring the entire robot stack with one command. This is much more convenient than starting each node manually."
  difficulty="easy"
/>

---

## Advanced Launch Features

### Including Other Launch Files

```python
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    other_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory('other_package'),
            '/launch/other_launch.py'
        ])
    )
    
    return LaunchDescription([
        other_launch,
        # ... your nodes ...
    ])
```

### Conditional Execution

```python
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    
    return LaunchDescription([
        Node(
            package='gazebo_ros',
            executable='gazebo',
            condition=IfCondition(use_sim_time)
        ),
        # ... other nodes ...
    ])
```

Run with:
```bash
ros2 launch my_package my_launch.py use_sim_time:=true
```

---

## What You've Learned

- ✅ **Parameters** allow runtime configuration without code changes
- ✅ Parameters can be set via command line, YAML files, or dynamically
- ✅ **Launch files** start and configure multiple nodes simultaneously
- ✅ Python launch files support conditionals, includes, and complex logic
- ✅ Best practice: Use parameters for configuration, launch files for orchestration

<Quiz
  question="How do you change a parameter value while a node is running?"
  options={[
    "Edit the source code and rebuild",
    "Use `ros2 param set /node_name parameter_name new_value`",
    "Restart the node with new command-line arguments",
    "Parameters cannot be changed while running"
  ]}
  correctAnswer={1}
  explanation="The `ros2 param set` command allows changing parameter values at runtime. If the node has a parameter callback registered, it can react to the change immediately without restarting."
  difficulty="medium"
/>

---

## Next Steps

You've learned all the core ROS 2 concepts! The final sub-chapter covers best practices, debugging, and production-ready patterns.

**Continue to**: [Best Practices & Debugging](08-best-practices)

---

## Additional Resources

:::info Dive Deeper
- [ROS 2 Parameters Tutorial](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Parameters/Understanding-ROS2-Parameters.html)
- [Launch File Tutorial](https://docs.ros.org/en/humble/Tutorials/Intermediate/Launch/Launch-Main.html)
:::
