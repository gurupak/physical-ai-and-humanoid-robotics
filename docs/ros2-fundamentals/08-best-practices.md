---
id: 08-best-practices
title: "Best Practices & Debugging"
sidebar_label: "Best Practices"
sidebar_position: 8
sidebar_custom_props:
  difficulty: "Intermediate"
  readingTime: "15 minutes"
  handsOnTime: "30 minutes"
---

import Quiz from '@site/src/components/Quiz';
import Callout from '@site/src/components/Callout';

# Best Practices & Debugging

Congratulations! You've learned all the core ROS 2 concepts. This final sub-chapter consolidates best practices, debugging techniques, and patterns for production-ready code.

---

## Naming Conventions

### Node Names

**Do**:
```python
self = Node('camera_driver')          # Lowercase, underscores
self = Node('object_detector_node')   # Descriptive
```

**Don't**:
```python
self = Node('Node1')                  # Generic
self = Node('cameraDriver')           # CamelCase
```

### Topic Names

**Do**:
```
/robot/camera/image_raw               # Hierarchical, namespaced
/cmd_vel                              # Standard ROS 2 convention
```

**Don't**:
```
/Image                                # Capitalized
/myTopic123                           # Non-descriptive
```

### Package Names

**Do**:
- `robot_bringup`, `my_robot_navigation`, `sensor_drivers`

**Don't**:
- `MyPackage`, `pkg1`, `RobotStuff`

<Callout type="tip" title="Standard ROS 2 Topic Names">
Follow established conventions:
- `/cmd_vel` - velocity commands
- `/odom` - odometry
- `/scan` - laser scan
- `/camera/image_raw` - camera images

This improves interoperability with existing packages.
</Callout>

---

## Code Structure Best Practices

### 1. Always Use Try-Finally for Cleanup

```python
def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
```

**Why**: Ensures cleanup happens even if errors occur.

### 2. Use Logger, Not Print

**Bad**:
```python
print(f'Publishing message: {msg.data}')
```

**Good**:
```python
self.get_logger().info(f'Publishing message: {msg.data}')
```

**Logger Levels**:
- `debug()` - Detailed diagnostic info
- `info()` - General informational messages
- `warn()` - Warning messages (non-critical)
- `error()` - Error messages (critical)
- `fatal()` - Fatal errors (node cannot continue)

### 3. Validate Parameters

```python
def __init__(self):
    super().__init__('my_node')
    
    self.declare_parameter('max_speed', 1.0)
    max_speed = self.get_parameter('max_speed').value
    
    # Validate range
    if max_speed <= 0 or max_speed > 10.0:
        self.get_logger().error(f'Invalid max_speed: {max_speed}. Must be 0-10.')
        raise ValueError('Invalid parameter')
```

### 4. Use Type Hints

```python
from rclpy.node import Node
from std_msgs.msg import String

class MyPublisher(Node):
    def __init__(self) -> None:
        super().__init__('publisher')
        self.publisher_: Publisher = self.create_publisher(String, 'topic', 10)
        
    def publish_message(self, data: str) -> None:
        msg = String()
        msg.data = data
        self.publisher_.publish(msg)
```

<Quiz
  question="Why should you use `self.get_logger().info()` instead of `print()` in ROS 2 nodes?"
  options={[
    "Loggers are faster than print statements",
    "Loggers provide severity levels, timestamps, and integration with ROS 2 logging tools",
    "Print statements are not supported in Python 3",
    "Loggers use less memory"
  ]}
  correctAnswer={1}
  explanation="ROS 2 loggers provide severity levels (info/warn/error), automatic timestamps, node name prefixes, and integration with tools like rqt_console for filtering and analysis. Print statements bypass all of this infrastructure."
  difficulty="easy"
/>

---

## Debugging Tools

### 1. `ros2 node` - Node Inspection

```bash
# List running nodes
ros2 node list

# Get node info (publishers, subscribers, services, actions)
ros2 node info /my_node
```

**Example Output**:
```
/my_node
  Subscribers:
    /parameter_events: rcl_interfaces/msg/ParameterEvent
  Publishers:
    /rosout: rcl_interfaces/msg/Log
    /chatter: std_msgs/msg/String
  Service Servers:
    /my_node/describe_parameters: ...
  Service Clients:

  Action Servers:

  Action Clients:

```

### 2. `ros2 topic` - Topic Inspection

```bash
# List topics
ros2 topic list

# Show message rate
ros2 topic hz /chatter

# Show bandwidth
ros2 topic bw /camera/image

# Echo messages with message rate
ros2 topic echo /chatter --once
```

### 3. `rqt_graph` - Visualize Nodes and Topics

```bash
rqt_graph
```

This opens a GUI showing:
- All nodes (ovals)
- All topics (rectangles)
- Connections (arrows)

**Use Case**: Quickly verify your system's communication graph.

### 4. `ros2 bag` - Record and Playback

Record messages for later analysis:

```bash
# Record all topics
ros2 bag record -a

# Record specific topics
ros2 bag record /camera/image /odom

# Playback
ros2 bag play my_bag/
```

<Callout type="tip" title="Debugging with Bag Files">
Record a problematic scenario with `ros2 bag record`, then play it back repeatedly while testing fixes. This is invaluable for intermittent bugs!
</Callout>

---

## Common Debugging Scenarios

### Scenario 1: Publisher/Subscriber Not Communicating

**Symptoms**: Subscriber receives no messages even though publisher is running.

**Debug Steps**:

1. **Check both nodes are running**:
   ```bash
   ros2 node list
   ```

2. **Verify topic names match**:
   ```bash
   ros2 topic list
   ros2 topic info /chatter
   ```
   
3. **Check QoS compatibility**:
   ```bash
   ros2 topic info /chatter -v
   ```
   
   Look for QoS mismatches (e.g., publisher is BEST_EFFORT, subscriber is RELIABLE).

4. **Echo the topic**:
   ```bash
   ros2 topic echo /chatter
   ```
   
   If you see messages here but subscriber doesn't, the issue is in subscriber code.

### Scenario 2: Node Crashes with "rcl not initialized"

**Cause**: Forgot to call `rclpy.init()` before creating nodes.

**Fix**:
```python
def main(args=None):
    rclpy.init(args=args)  # ← Must be first
    node = MyNode()
    rclpy.spin(node)
```

### Scenario 3: Service Call Hangs Forever

**Cause**: Service server not running or wrong service name.

**Debug**:
```bash
# Check if service exists
ros2 service list

# Try calling from CLI
ros2 service call /add_two_ints example_interfaces/srv/AddTwoInts "{a: 2, b: 3}"
```

---

## Performance Best Practices

### 1. Choose Appropriate Queue Sizes

```python
# High-frequency sensor data (allow dropping old messages)
self.publisher_ = self.create_publisher(Image, '/camera/image', 10)

# Critical commands (don't lose messages)
self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 100)
```

**Rule of Thumb**: Queue size = publish rate × expected max delay

### 2. Avoid Heavy Computation in Callbacks

**Bad**:
```python
def image_callback(self, msg):
    # Heavy processing (blocks executor)
    processed = self.run_neural_network(msg)
    self.publish_result(processed)
```

**Good**:
```python
def image_callback(self, msg):
    # Queue message for processing
    self.image_queue.put(msg)

# Process in separate thread
def processing_thread(self):
    while rclpy.ok():
        msg = self.image_queue.get()
        processed = self.run_neural_network(msg)
        self.publish_result(processed)
```

### 3. Use Composition for Low Latency

Instead of separate processes, run multiple nodes in one process:

```python
from rclpy.executors import SingleThreadedExecutor

executor = SingleThreadedExecutor()
executor.add_node(node1)
executor.add_node(node2)
executor.spin()
```

---

## Security Best Practices

### 1. Never Hardcode Credentials

**Bad**:
```python
API_KEY = "12345-secret-key"
```

**Good**:
```python
import os
API_KEY = os.getenv('ROBOT_API_KEY')
if not API_KEY:
    raise ValueError('ROBOT_API_KEY environment variable not set')
```

### 2. Validate External Input

```python
def command_callback(self, msg):
    # Validate velocity is within safe limits
    if abs(msg.linear.x) > MAX_SPEED:
        self.get_logger().warn(f'Velocity {msg.linear.x} exceeds max {MAX_SPEED}')
        return
    
    self.send_to_motors(msg)
```

---

## What You've Learned

-  ✅ **Naming conventions** for nodes, topics, packages
- ✅ **Code structure** best practices (cleanup, logging, validation)
- ✅ **Debugging tools** (`ros2 node`, `ros2 topic`, `rqt_graph`, `ros2 bag`)
- ✅ **Common debugging scenarios** and solutions
- ✅ **Performance tips** (queue sizes, threading, composition)
- ✅ **Security practices** (no hardcoded secrets, input validation)

<Quiz
  question="What tool would you use to visualize the connections between nodes and topics in your ROS 2 system?"
  options={[
    "ros2 node list",
    "rqt_graph",
    "ros2 topic echo",
    "ros2 bag"
  ]}
  correctAnswer={1}
  explanation="`rqt_graph` provides a visual representation of the ROS 2 computational graph, showing nodes, topics, and their connections. This is invaluable for understanding and debugging complex systems."
  difficulty="easy"
/>

---

## Chapter Summary

**Congratulations!** 🎉 You've completed Chapter 1: ROS 2 Fundamentals. You now understand:

1. **Overview**: What ROS 2 is and why it's revolutionary
2. **Installation**: Setting up ROS 2 Humble
3. **Your First Node**: Creating publishers
4. **Topics & Messages**: Asynchronous pub-sub communication
5. **Services**: Synchronous request-response
6. **Actions**: Long-running tasks with feedback
7. **Parameters & Launch**: Configuration and orchestration
8. **Best Practices**: Production-ready patterns and debugging

### Where to Go Next

You're ready to build real robotic applications! Consider exploring:

- **Chapter 2**: Simulation with Gazebo and Isaac Sim
- **Chapter 3**: Robot perception (cameras, lidar, computer vision)
- **Chapter 4**: Navigation and path planning
- **Chapter 5**: Manipulation and control

---

## Final Quiz

<Quiz
  question="Which communication pattern would you use for a task that takes 30 seconds, provides progress updates every 2 seconds, and can be canceled midway?"
  options={[
    "Topic (publish-subscribe)",
    "Service (request-response)",
    "Action (goal-feedback-result)",
    "Parameter"
  ]}
  correctAnswer={2}
  explanation="Actions are designed for long-running tasks that provide periodic feedback and support cancellation. Topics don't provide responses, services don't support feedback or cancellation, and parameters are for configuration, not tasks."
  difficulty="hard"
/>

---

## Additional Resources

:::info Continue Learning
- [ROS 2 Documentation](https://docs.ros.org/en/humble/)
- [ROS 2 Tutorials](https://docs.ros.org/en/humble/Tutorials.html)
- [ROS Discourse Forum](https://discourse.ros.org/)
- [awesome-ros2](https://github.com/fkromer/awesome-ros2) - Curated list of ROS 2 resources
:::

**Thank you for completing Chapter 1!** You're well on your way to becoming a ROS 2 developer. 🤖
