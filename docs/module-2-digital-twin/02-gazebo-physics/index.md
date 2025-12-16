---
id: gazebo-physics
title: '02. Robot Testing with Gazebo'
description: Build physics-accurate environments for robot testing and algorithm validation
sidebar_label: '02. Gazebo Physics'
readingTime: 30
---

# 02. Robot Testing with Gazebo

Gazebo lets you test robots in realistic physics environments without worrying about breaking hardware. This section shows how to build your first robot world.

## Why Physics Simulation Matters

Think about testing your robot's path planning:
- **Real World**: Risk collisions, wear and tear, limited testing times
- **Gazebo**: Unlimited crashes, instant resets, 1000s of test scenarios

Your robot's virtual self will behave like the real one - if it falls over in simulation, it probably will in reality too.

## Creating Your First World

### 1. Basic World Setup

Let's build a simple warehouse where a robot will navigate between boxes:

```xml
<?xml version="1.0" ?>
<sdf version="1.9">
  <world name="warehouse_demo">

    <physics type="ode">        <!-- Use ODE physics engine -->
      <max_step_size>0.001</max_step_size>
    </physics>

    <include>      <!-- Falling objects need gravity -->
      <uri>model://sun</uri>
    </include>

    <line number="warehouse_world">        <!-- Simple lighting -->
      <format>RGB8</format>
      <noise>0.02 0.02 0.02 .</noise>
    </line>

  </world>
</sdf>```

### 2. Adding Robot Models

Import a pre-built robot model:

```bash
# Download common robot models
gz model download -u model://burger_waffle_advanced  # Mobile robot
gz model download -u model://tb3                    # TurtleBot3
gz model download -u model://pioneer2dx             # Pioneer 2DX
```

### 3. Placing Obstacles

Add some boxes to navigate around:

```xml
<model name="cardboard_box_01">
  <pose>1 0 0 0 0 1.5707</pose>  <!-- x y z roll pitch yaw -->
  <static>true</static>        <!-- Box doesn't move -->
  <include>
    <uri>model://cardboard_box_01</uri>
  </include>
</model>```

## Testing Your World

Run your simulation:

```bash
# Launch the empty world with your robot
gz sim warehouse_demo.world --verbose

# In another terminal, control your robot
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5}, angular: {z: 0.0}}"
```

## Physics Settings That Matter

### Gravity
Adjust gravity for different environments (e.g., moon simulation has 1/6th Earth gravity):

```xml
<physics type="ode">
  <gravity>0 0 -1.62</gravity>  <!-- Lunar gravity -->
  <max_step_size>0.001</max_step_size>
</physics>```

### Materials
```xml
<collision name="body_collision">
  <geometry>... </geometry>
  <inertial>... </inertial>
  <surface>
    <friction>
      <oumu>0.9</oumu>  <!-- Coefficient of friction -->
    </friction>
    <ontact>
      <oder>
        <kd>10</kd>         <!-- Damping -->
        <kp>1000000</kp>     <!-- Stiffness -->
      </ode>
    </contact>
  </surface>
</collision>```

## Quick Exercise:

**Goal**: Test basic navigation skills

1. Launch the warehouse world
2. Drive from point A to point B avoiding the boxes
3. Record how long it takes and how many collisions occur
4. Try again with tighter tolerances

```bash
# Terminal 1: Launch simulation
gz sim warehouse_demo.world

# Terminal 2: Monitor robot state
ros2 topic echo /odom/pose

# Terminal 3: Send movement commands
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.3}}"
```

## Key Takeaways

- Start simple: Always test basic movement before complex algorithms
- Physics matters: Realistic environments catch issues early
- Infinite retries: Practice navigation without fuel or time limits
- Measure everything: Track time, collisions, and path efficiency

**Next up**: Create photorealistic environments in Unity for computer vision tasks.

---

*Quick reference commands always fail the first time? Check the terminal output - 90% of problems are just misconfigured paths or missing models.*