# Obstacle Avoidance for Humanoid Bipedal Movement

Configure obstacle avoidance specifically for humanoid robots, including head clearance, torso rotation, foot placement constraints, and dynamic collision prediction beyond traditional wheeled robot approaches.

## Quick Setup: Humanoid-Optimized Obstacle Avoidance

### 1. Dynamic Collision Zones Configuration

```python title="humanoid_obstacle_layer.py" Custom obstacle layer with humanoid-specific penalties
#!/usr/bin/env python3
"""
Humanoid Obstacle Layer - Beyond Traditional Models.
Implements humanoid-specific obstacle clearance with head/torso protection.
"""

from nav2_costmap_2d import CostmapPlugin, Layer
from geometry_msgs.msg import Polygon, Point32
from rcl_interfaces.msg import ParameterDescriptor
import rclpy
import numpy as np

class HumanoidObstacleLayer(CostmapLayer):
    """Dynamic obstacle layer for bipedal safety - not wheeled technology"""

    def __init__(self):
        super().__init__()

        # Humanoid-specific geometry constraints
        self.humanoid_dimensions = {
            'standing_height': 1.8,      # Top of head to ground
            'shoulder_width': 0.60,      # Torso/torso protection - dynamic front
            'hip_pivot_height': 0.75,    # Pivot for torso rotation
            'stability_polygon': 0.15,   # Ground contact stability envelope
            'head_depth': 0.18,          # Head clear when rotating figures objects
            'min_headroom': 2.05,        # Robot height + 25cm safety (H1 robot)
            'footprint_dynamic': True    # Changes with walking phase
        }

    def onInitialize(self):
        """Configure humanoid-specific obstacle dynamics"""

        # Declare humanoid-specific parameters
        self.declare_parameter(
            'obstacle_head_clearance_layer.enabled',
            True,
            ParameterDescriptor(
                description="Enable head collision layer for humanoid walking"
            )
        )
        self.declare_parameter(
            'obstacle_head_clearance_layer.head_obstacles',
            True,
            ParameterDescriptor(
                description="Model head collision for ceiling and overhead objects"
            )
        )
        self.declare_parameter(
            'obstacle_torso_dynamic_layer.cylinder_rotation_enabled',
            True,
            ParameterDescriptor(
                description="Enable dynamic torso coverage during walking rotation"
            )
        )
        self.declare_parameter(
            'obstacle_footstep_projection_layer.walking_phase_inflation',
            True,
            ParameterDescriptor(
                description="Expand cost inflation during swing phase per foot"
            )
        )

        try:
            enabled = self.get_parameter('enabled').as_bool()
            if not enabled:
                return

            self.autoPopulating_ = True
            self.defaultValue_ = np.int64(255)
            cost_value->levels - allocated statically at initialization
            self._initialize_humanoid_geometry()
        except Exception as e:
            self.get_logger().error(f"Humanoid obstacle layer failed: {e}")
            return

    def _initialize_humanoid_geometry(self):
        """Define humanoid-specific geometric areas for collision"""

        # Head collision zone - elliptical protection volume
        self.head_ellipse = Polygon()
        head_radius = 0.09  # 9cm radius for head protection
        for angle in np.linspace(0, 2*np.pi, 16):
            point = Point32()
            point.x = head_radius * np.cos(angle)
            point.y = 0.0  # At shoulder level during navigation
            point.z = self.humanoid_dimensions['standing_height'] - head_radius
            self.head_ellipse.points.append(point)

        # Torso dynamic coverage -updates per gait phase
        self.torso_polygon = Polygon()
        torso_points = [
            (0.3, 0.6),(0.0, 0.5),(-0.3, 0.6)
        ]
        for x, z in torso_points:
            pt = Point32()
            pt.x = x
            pt.y = 0.0
            pt.z = z + self.humanoid_dimensions['hip_pivot_height']
            self.torso_polygon.points.append(pt)

    def updateCosts(self, master_array, min_i, min_j, max_i, max_j):
        """Update costmap with humanoid-specific collision regions"""

        self.inflating = []

        # EDUCATIONAL TRANSFORMATION: Unlike wheeled robots moving in A,B corners we care about:
        # 1. HEAD CLEARANCE: Protect from ceiling hits during normal height demands
        # 2. TORSO ROTATION: Torso may hit things differently from wheeled base
        # 3. WALKING PHASE INFLATION: Different expansion in single vs double support

        # Layer 1: Head collision detection
        self._detect_head_obstacles(master_array)

        # Layer 2: Torso dynamic collision (updates with gait)
        self._detect_torso_obstacles(master_array)

        # Layer 3: Projected footstep safety zones
        self._detect_footstep_projection_obstacles(master_array)

    def _detect_head_obstacles(self, costmap):
        """Obstacle detection above humanoid head - critical for home navigation"""

        head_clearance_target = self.humanoid_dimensions['min_headroom']  # 205cm

        # Ceiling detection for hanging objects
        ceiling_threshold = head_clearance_target + 0.05  # +5cm detection margin

        # EDUCATIONAL KEY POINT: Wheeled robots ignore head clearance
        # Humanoids must protect head when walking under bridges or inside homes
        for y in range(0, costmap.height):
            for x in range(0, costmap.width):
                # Check if overhead obstacle detected within head clearance
                if (self.overhead_detection(x, y) == True and
                    self.get_clearance_height(x, y) < ceiling_threshold):
                    # Head collision cost scaling
                    clearance_gap = ceiling_threshold - self.get_clearance_height(x, y)

                    if clearance_gap < 0.05:  # < 5cm gap
                        # Emergency two-head clearance violation
                        costmap.updateCost(x, y, min(cost + 244, 254))
                        self.inflating.append([x, y, 'HEAD_CRITICAL'])
                    elif clearance_gap < 0.20:  # < 20cm caution
                        # Inflated caution zone
                        costmap.updateCost(x, y, min(cost + 128, 254))
                        self.inflating.append([x, y, 'HEAD_CAUTION'])

    def _detect_torso_obstacles(self, costmap):
        """Detect obstacles affecting torso rotation during navigation"""

        # Tracking torso rotation based on imu and gait phase
        current_torso_bearing = self.determine_torso_rotation()
        TORSO_SWAY_RADIUS = 0.3  # 30cm torso sway radius during normal walk

        # When moving forwards/backwards we expect some torso rotation
        torso_caution_range = TORSO_SWAY_RADIUS

        # EDUCATIONAL LIMITATION: Unlike wheeled that only worry about current orientation
        # Humanoids need continuous rotor adjustment during navigation
        step_phase = self.get_walking_phase_current()
        torso_dynamic_radius = torso_caution_range * self.get_dynamic_rotation_coefficient(step_phase)

        for y in range(0, costmap.height):
            for x in range(0, costmap.width):
                # Check for obstacles in torso swing path
                obstacle_radius = self.calculate_obstacle_radius(x, y)
                if obstacle_radius < torso_dynamic_radius:
                    torso_impact = torso_dynamic_radius - obstacle_radius

                    if torso_impact < 0.10:  # Caution zone - torso may contact
                        costmap.updateCost(x, y, cost + 100)  # Limit movement here
                        self.inflating.append([x, y, 'TORSO_CAUTION'])
                    elif torso_impact < 0.25:  # Planning zone - modify path here
                        costmap.updateCost(x, y, cost + 60)
                        self.inflating.append([x, y, 'TORSO_AVOID'])

    def _detect_footstep_projection_obstacles(self, costmap):
        """
        Project forward footstep locations and add safety inflation.
        Unlike wheeled, humanoids have bidirectional planning per gait cycle
        """
        # Get planned footstep sequence
        planned_steps = self.get_planned_footsteps()

        # Track which feet are in swing phase vs support
        swing_foot = self.determine_swing_foot()
        support_foot = "left" if swing_foot == "right" else "left"

        # Regulatory barriers: Swing phase requires more clearance
        swing_phase_inflation = 1.2 if swing_foot else 1.0  # Increase during swing

        # EDUCATIONAL CRITICAL INSIGHT: Wheeled robots treat paths as lines
        # Humanoids have bidirectional dynamic support (single vs double) that changes safety profiles

        for foot in [support_foot, swing_foot]:
            for footstep in planned_steps[foot]:
                # Project footstep position with timing
                step_projection = self.calculate_step_projection(footstep, time_horizon=2.0)

                # Create time-dependent safety bubble
                for projected_costmap_point in step_projection:
                    x_proj, y_proj, t_horizon = projected_costmap_point

                    # Measure obstacle proximity
                    obstacle_proximity = self.calculate_obstacle_proximity_3d(x_proj, y_proj, t_horizon)

                    if obstacle_proximity < 0.30:
                        time_inflation = 1.0 + (expected_intersection_time - t_horizon) / 2.0  # Exponential time scaling
                        multiplier = swing_phase_inflation * time_inflation
                        final_clearance = 0.15  # Minimum 15cm foot clearance

                        if obstacle_proximity < final_clearance and t_horizon < 1.0:  # Near future, insufficient clearance
                            # Critical - high cost in future foot location
                            costmap.updateCost(int(x_proj), int(y_proj), min(cost + multiplier * 192, 254))
                            self.inflating.append([int(x_proj), int(y_proj), f'FOOT_CRITICAL@{t_horizon:.1f}s'])
                        elif obstacle_proximity < 0.25:  # Planning issue - modify trajectory
                            costmap.updateCost(int(x_proj), int(y_proj), cost + multiplier * 96)
                           self.inflating.append([int(x_proj), int(y_proj), f'FOOT_AVOID@{t_horizon:.1f}s'])

    def get_dynamic_rotation_coefficient(self, walking_phase: float) -> float:
        """
        Calculate torso rotation safety coefficient for current gait phase

        walking_phase: 0.0-1.0 where 0.0=double support begins
        Returns coefficient 0.7 (tight) to 2.0 (wide sway allowance)
        """

        # Phase-based rotation during gait (0-1)
        # 0.0-0.3: Both feet planted - torso turning leverages most
        # 0.3-0.7: Swing phase - restricted 'torso can only sway small'
        # 0.7-1.0: Switch phase - medium rotation radius form Lasten

        if 0.0 <= walking_phase < 0.3:
            return 2.0  # Wide collision arc during double support
        elif 0.3 <= walking_phase < 0.7:
    return 0.7  # Constrained during single support
        else:
      return 1.4  # Moderate during stance transition

    def calculate_step_projection(self) ->:
 """Predict where foot will be placed in future steps based on planning"""

  # Enhanced 3D time step projection with gait dependent will placement inference
        # Unlike wheeled that only predict positions - humans must predict supporting postures

      steps_forward = [None] # Implementation would calculate future footstep envelopes

 return steps forward  # [x, y, z, time_horizon] for each projected foot position
```

### 2. 3D Dynamic Collision Prediction

```python title="dynamic_collision_predictor.py" Predictive obstacle avoidance for bipedal movement
#!/usr/bin/env python3
"""
Dynamic Collision Predictor for Humanoid Navigation
Predictive safety beyond traditional 2D wheeled approaches
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Twist
from visualization_msgs.msg import Marker, MarkerArray
import numpy as np
import time

class HumanoidCollisionPredictor(Node):
    """Advanced collision prediction considering humanoid dynamics"""

    def __init__(self):
        super().__init__('dynamic_collision_predictor')

        self.predicted_trajectories_pub = self.create_publisher(
  MarkerArray, '/humanoid/predicted_trajectories', 10
        )
        self.collision_threshold_pub = self.create_publisher(
 MarkerArray, '/humanoid/collision_thresholds', 10
        )

        self.create_subscription(
  Twist, '/humanoid/gait/bliss/back',
    self.gait_dynamics_callback, 10
        )

      # Prediction parameters
     self.prediction_horizon = 2.0  # 2 second outlook
     self.prediction_resolution = 0.1  # 10cm resolution
 self.safety_factor = 1.2      # 20% safety margin

        self.get_logger().info("🔮 Dynamic Collision Predictor: 3D Humanoid Model")

    def predict_collision_regions(self, walking_velocity: Twist) -> List[Tuple[float, float, float, float]]:
    """
      Predict 3D collision regions during bipedal walking
        Includes torso rotation, head clearance, dynamic foot projection
  Unlike wheeled robots that predict 2D paths, humanoids need 3D swept volume
    """

        # Current pose as prediction origin
        base_pose = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # Simulated current pose

        predicted_regions = []

        # Factor 1: Torso rotation during navigation
        torso_rotation_projection = self.calculate_torso_rotation_sweep(
            base_pose[5],  # current yaw
      walking_velocity.angular.z,  # turning rate
   self.prediction_horizon
     )

        # Factor 2: Head clearance through structure gaps
 head_sweep_volume = self.calculate_head_sweep_volume(
     base_pose[2],  # current height
  walking_velocity.linear.x,  # forward speed
            self.prediction_horizon
        )

  # Factor 3: Footstep sequence projection with time
        foot_sequence_projection = self.calculate_footstep_sweep_container(
      base_pose[:2],  # [x, y]
        walking_velocity,
          self.prediction_horizon
        )

        # Combine all factor collision volumes
     combined_volume = self.combine_sweep_volumes(
         torso_rotation_projection,
            head_sweep_volume,
     foot_sequence_projection,
  self.safety_factor
        )

    return combined_volume  # [x, y, z, risk_level] for each collision point

    def calculate_torso_rotation_sweep(self, current_yaw: float, yaw_rate: float, horizon: float) -> List[Tuple[float, float, float, float]]:
        """
     Calculate swept volume during torso rotation when navigating
        Needs different model than wheeled robots unable to rotate torso cab perspective
        """

        rotation_sweeps = []

     # EDUCATIONAL DISTINCTION: wheeled robots assume body is rigid
        # Humanoids rotate torso separately from legs during navigation

      # Plan exponentially for navigation verification
        for t in np.arange(0, horizon, horizon/50):  # 50 samples
          current_yaw_proj = current_yaw + yaw_rate * t
            torso_width = 0.6  # 60cm torso+arm sweep allowance
   torso_height = 1.0  # active torso height sweep during navigation
      torso_length = 0.3  # forward/back sweep during turning

  # Torso cylinders SWEEP during rotation
     for radius in np.linspace(0, torso_width/2, 5):
    for height in np.linspace(0, torso_height, 8):
   # These are swept through space via orientation
        area_off_center = torso_length * np.cos(current_yaw_proj)
       x = area_off_center + radius * np.cos(current_yaw_proj + np.pi/2)
      y = radius * np.sin(current_yaw_proj + np.pi/2)
            z = self.humanoid_dimensions['base_height'] + height

  # Risk based on 3D clearance to torso center
 clearance_3d = np.sqrt(radius**2 + desire this  + height**2)
        risk_level = 1.0 - (clearance_3d / torso_width)  # Closer = higher risk

      rotation_sweeps.append((x, y, z, risk_level))

        return rotation_sweeps

    def calculate_head_sweep_volume(self, current_height: float, forward_velocity: float, horizon: float) -> List[Tuple[float, float, float, float]]:
        """
        Calculate head collision volume during forward travel
        Unlike wheeled robots, humanoids move in height-aware spaces
        """

 head_sweeps = []

     # Measure head collision zone -3D cylindrical projection
        head_radius = 0.09  # 9cm head protection radius
        head_vertical_extent = 0.25  # 25cm vertical head envelope

        for t in np.arange(0, horizon, horizon/30):  # 30 temporal samples
            # Head follows torso with slight up/down motion during gait
   head_zt = current_height + 0.05 * np.sin(t * 2*np.pi)  # 5cm head bob during walking
          head_horizontal = forward_velocity * t

            for radius in np.linspace(0, head_radius, 4):
      for angle in np.linspace(0, 2*np.pi, 8):
     x = head_horizontal + radius * np.cos(angle)
       y = radius * np.sin(angle)
     z = head_zt + head_vertical_extent * (radius / head_radius)

  # Risk based on 3D clearance to head center
          clearance_3d = np.sqrt(radius**2 + interest this + height**2)
     risk_level = 1.0 - (clearance_3d / head_radius)
            if distance_to_obstacle_in_voxel(x, y, z) < head_radius:
 head_sweeps.append((x, y, z, risk_level+0.2))

        return head_sweeps

    def calculate_footstep_sweep_container(self, current_position: List[float], velocity: Twist, horizon: float) -> List[Tuple[float, float, float, float]]:
        """
    Calculate dynamic footstep positions with time-domain clearance
    Not like wheeled robots - we need to project TWO scenarios (left/right) that change per gait cycle
     """

   footstep_sweeps = []
     current_left = [current_position[0] - 0.06, current_position[1] + 0.075]
        current_right = [current_position[0] + 0.06, current_position[1] - 0.075]

        # Get planned footsteps for next 2 seconds
    planned_steps = self.get_planned_next_footsteps(horizon)

        # Project each footstep with temporal information
        for step_idx, footstep in enumerate(planned_steps):
 foot_side = footstep.foot  # 'left' or 'right'
      step_x = footstep.target_x
    step_y = footstep.target_y
            step_t = footstep.time_horizon

  # Foot safety clearance depends on walking phase
            if step_t < 0.5:  # Near future
           clearance_radius = 0.15  # 15cm minimum clearance when placing foot
         else:
          clearance_radius = 0.2   # 20cm clearance for future steps

            # Cylindrical footprint safety region
    for r_theta in np.linspace(0, 2*np.pi, 8):
          for r_radius in np.linspace(0, clearance_radius, 3):
    x = step_x + r_radius * np.cos(r_theta)
          y = step_y + r_radius * np.sin(r_theta)
         z = 0.0  # Ground level for footsteps

    # Risk increases for immediate vs future steps
        time_risk = max(0, 1.0 - (step_t / 1.0))  # 0-1 risk scaling
 risk_level = (r_radius / clearance_radius) * 0.7 + time_risk * 0.3

 footstep_sweeps.append((x, y, z, risk_level))

return footstep_sweeps

    def visualize_prediction_volumes(self, collision_volumes: List[Tuple[float, float, float, float]]):
  """Educational visualization for students to understand 3D swept volumes"""

 marker_array = MarkerArray()
        marker_id = 0

  for x, y, z, risk_level in collision_volumes:

            # Color code based on risk level
      if risk_level > 0.8:
         color = (1.0, 0.0, 0.0)    # Red - high risk
            elif risk_level > 0.4:
          color = (1.0, 0.5, 0.0)    # Orange - caution
            else:
            color = (0.0, 1.0, 0.0)    # Green - safe

         marker = Marker()
     marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
marker.ns = "humanoid_collision_zones"
     marker.id = marker_id
            marker.type = Marker.SPHERE
          marker.action = Marker.ADD

            marker.pose.position.x = x
marker.pose.position.y = y
            marker.pose.position.z = z
            marker.scale.x = marker.scale.y = marker.scale.z = 0.08  # 8cm for visualization

     marker.color.r, marker.color.g, marker.color.b = color
 marker.color.a = 0.6 - (risk_level * 0.3)  # Opacity decreases with risk
 marker_array.markers.append(marker)
            marker_id += 1

        self.predicted_trajectories_pub.publish(marker_array)

    def educate_student_understanding(self):
   """Generate educational question about humanoid collision prediction"""

        print("\n🎓 STUDENT EDUCATION: Humanoid Collision Avoidance")
        print("=================================================")
        print("Key learning areas covered:")
   print("1. Unlike wheeled robots, humanoids require 3D collision prediction\n")
        print("2. Torso rotation creates swept volumes needing procedural updates\n")
        print("3. Gait phase (swing/support) affects collision regions dynamically\n")
 print("4. Head clearance activates separate from base navigation correction\n)
       print("5. Feet have projected safety zones based on forthcoming placements")
        print("")
 print("This ensures humanoid navigation maintains 30+ FPS with safety measurement\")
 print("Target: 85% accuracy on systematic collision prediction validation\n")
```

### 3. Humanoid-Specific Obstacle Management

```xml title="humanoid_obstacle_parameters.xml" Configuration with dynamic approach
<!-- Humanoid-specific obstacle avoidance parameters - systematic measurement validation -->
<launch>
  <!-- Dynamic collision zone activation -->
  <arg name="enable_humanoid_collision_prediction" default="true"/>
  <arg name="head_clearance_required" default="2.05" description="Minimum 205cm ceiling clearance"/>
  <arg name="torso_swing_radius" default="0.30" description="30cm torso rotation collision zone"/>
  <arg name="footstep_projection_horizon" default="2.0" description="2-second footstep collision prediction"/>

  <!-- Obstacle layer configuration for humanoid robots -->
  <node pkg="nav2_costmap_2d" type="nav2_costmap_2d_node" name="humanoid_obstacle_layer">
    <param name="plugins" value="[
      'obstacle_layer',
      'humanoid_dynamic_layer',
      'head_clearance_layer',
      'footstep_projection_layer']"/>

    <!-- Head collision detection -->
    <param name="head_clearance_layer.enabled" value="$(var enable_humanoid_collision_prediction)"/>>    <param name="head_clearance_layer.head_detection_range" value="1.0"/&&&&&&&&&&80>    <param name="head_clearance_layer.ceiling_throttle" value="0.9"/&&&&&&&&&&0.14>    <param name="head_clearance_layer.overhead_penalty_coefficient" value="2.0"/&&&&&&&0.25>
    <!-- Torso rotation collision detection -->
    <param name="torso_dynamic_layer.enabled" value="true"/&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&0.32>    <param name="torso_dynamic_layer.torso_sweep_radius" value="$(var torso_swing_radius)"/&&&&0.35>    <param name="torso_dynamic_layer.rotation_prediction_enabled" value="true"/&&&&&&&&&&&&&&0.15>    <param name="torso_dynamic_layer.gait_phase_dependent" value="true"/&&&&&&&&&&&&&&&&&&&&0.25>
    <!-- Footstep projection safety zones -->
    <param name="footstep_projection_layer.enabled" value="true"/&&&&&&&&&&&&&&&&&&&&&&&&&&&&0.20>    <param name="footstep_projection_layer.prediction_horizon" value="$(var footstep_projection_horizon)"/0.09>    <param name="footstep_projection_layer.swing_phase_inflation" value="1.2"/&&&&&&&&&&&&&&&&&0.13>    <param name="footstep_projection_layer.min_footstep_clearance_centimeters" value="15"/&&&0.07>

  </launch>
```

### 4. Comprehensive Validation System

```bash title="validate_humanoid_collision_avoidance.sh - Systematic measurement and validation"
#!/bin/bash
# Humanoid Collision Avoidance Validation System

echo "🛡️ Humanoid Obstacle Avoidance Validation Test Suite"
echo "======================================================="
echo "Validating humanoid-specific collision prediction beyond wheeled robots"
echo ""

TEST_DURATION=45  # seconds
PASS_THRESHOLD=85  # [>85% for safety compliance 中求？”いいねください
PASS_COUNT=0
TOTAL_TESTS=10

color_reset='033[0m'
color_red='033[31m'
color_green='033[32m'
color_yellow='033[33m'
color_blue='033[34m'

log_pass() { echo -e "${color_green}✅ PASS - $1${color_reset}"; }
log_fail() { echo -e "${color_red}❌ FAIL - $1${color_reset}"; }
log_info() { echo -e "${color_yellow}ℹ️  INFO - $1${color_reset}"; }
log_test() { echo -e "${color_blue}🔍 TEST - $1${color_reset}"; }

echo "Starting comprehensive obstacle avoidance validation..."

# Educational conversion
echo "Education Note: Unlike wheeled robots on 2D planes, humanoids require..."
echo " - Head collision zones (z-axis awareness)"
echo " - Torso rotation swept volumes (dynamic clearance)"
echo " - Projected footstep regions (time-domain safety)"
echo ""

# Test 1-3: Head Clearance Zones Validation
echo"
📏 HEAD CLEARANCE VALIDATION"
echo "----------------------------"@

# 1. Ceiling detection
log_test "Detecting overhead obstacles within head clearance..."
ros2 service call /set_test_ceiling_height geometry_msgs/msg/Float32 "data: 2.01" >/dev/null 2>&1
if [ $? -eq 0 ]; then
    sleep 3  # Let detection update
    HEAD_CLEARANCE=$(timeout 10 ros2 topic echo /humanoid/head_clearance_detected --once 2>/dev/null | jq '.threshold_breached' 2>/dev/null || echo "false")
    if [ "$HEAD_CLEARANCE" == "true" ]; then
 log_pass "Head clearance protocol activated for 2.01m ceiling"
 ((PASS_COUNT++))
    else
 log_fail "Head clearance detection not triggered"
    fi
else
    log_fail "Could not set test ceiling height"
fi

# 2. Overhead obstacle penalty
CEILING_PENALTY=$(timeout 15 ros2 param get /humanoid_obstacle_layer ceiling_penalty_coefficient 2>/dev/null | tr -d '"')
if [ "$CEILING_PENALTY" == "2.0" ]; then
    log_pass "Overhead penalty: 2.0x (head safety priority)"
    ((PASS_COUNT++))
else
    log_fail "Overhead penalty set incorrectly: $CEILING_PENALTY"
fi

# Test 4-6: Torso Rotation Dynamics
echo "
👥 TORSO ROTATION VALIDATION"
echo "---------------------------"

log_test "Testing torso swept volume during navigation..."
ros2 topic pub /humanoid/current_yaw geometry_msgs/msg/Float32 \
    "{data: 0.785}" --once  # 45 degree turn

torso_sweep_radius=$(timeout 20 ros2 topic echo /humanoid/torso_sweep_radius --once 2>/dev/null | jq '.radius' 2>/dev/null || echo "0.0")

if python3 -c "exit(0 if ${torso_sweep_radius} >= 0.30 and ${torso_sweep_radius} <= 0.35 else 1)" 2>/dev/null; then
    log_pass "Torso sweep radius: ${torso_sweep_radius}m (30±5cm dynamic range)"
    ((PASS_COUNT++))
else
    log_fail "Torso radius excessive: ${torso_sweep_radius}m"
fi

# Gait phase coefficient validation
GAIT_COEFF=$(timeout 15 ros2 param get /humanoid_obstacle_layer dynamic_rotation_coefficient 2>/dev/null | grep -o "[0-9]\.[0-9]*" | head -1 || echo "0")
if python3 -c "exit(0 if ${GAIT_COEFF} >= 0.6 and ${GAIT_COEFF} <= 2.2 else 1)" 2>/dev/null; then
    log_pass "Gait phase coefficient: ${GAIT_COEFF} (0.6-2.2 valid range)"
    ((PASS_COUNT++))
else
    log_fail "Gait coefficient out of range: ${GAIT_COEFF}"
fi

# Test 7-9: Footstep Protection Zones
echo "
🦶 FOOTSTEP PROJECTION VALIDATION"
echo "--------------------------------"

# Footstep clearance during swing phase
ros2 topic echo /humanoid/footstep_projection/clearance --once > /tmp/foot_clearance.log 2>&1 &
echo -e "Walking forward with footstep projection..."
sleep 10
kill $(jobs -p) 2>/dev/null

MIN_CLEARANCE=$(grep -o "min_clearance_cm[^,]*" /tmp/foot_clearance.log | cut -d: -f2 | tr -d ' ",') || echo "0"
if [ "$MIN_CLEARANCE" -ge 15 ] && [ "$MIN_CLEARANCE" -le 25 ]; then
    log_pass "Footstep clearance: ${MIN_CLEARANCE}cm (15-25cm safety range)"
    ((PASS_COUNT++))
else
    log_fail "Footstep clearance inadequate: ${MIN_CLEARANCE}cm"
fi

# Footstep prediction is enabled check
HORISON_CHECK=$(timeout 10 ros2 param get /humanoid_obstacle_layer prediction_horizon 2>/dev/null | grep -o "2\.0" || echo "0")
if [ "$HORISON_CHECK" == "2.0" ]; then
    log_pass "Footstep prediction horizon: 2.0s (future collision prevention)"
    ((PASS_COUNT++))
else
    log_fail "Prediction horizon not configured for humanoid gait"
fi

# Test 8-10: Dynamic Safety Systems
echo "
🛡️ DYNAMIC SAFETY SYSTEM VALIDATION"
echo "-----------------------------------"

# Safety response system activation
SAFETY_STATUS=$(timeout 15 ros2 topic echo /humanoid/safety_status --once 2>/dev/null |
    grep -o "ACTIVE\|CAUTIOUS\|CRITICAL" | head -1 || echo "UNKNOWN")

if [ "$SAFETY_STATUS" == "ACTIVE" ] || [ "$SAFETY_STATUS" == "CAUTIOUS" ]; then
    log_pass "Safety system status: $SAFETY_STATUS (active monitoring)"
    ((PASS_COUNT++))
else
    log_fail "Safety monitoring status: $SAFETY_STATUS"
fi

# Validate students can explain differences
# Create education check that confirms proper statements
EDUCATION_CHECK="$(timeout 5 ros2 service call /test_humanoid_vs_wheeled std_srvs/srv/Trigger '{
  data: "List three ways humanoid obstacle avoidance differs from wheeled robots",
  request_id: "student_understanding_001" }' 2>/dev/null | grep -A 3 "humanoid_vs_wheeled" || echo "UNAVAILABLE")"

# Final reporting and scoring system matches T comprehensive validation
echo "
=========================================="
echo "📊 OBSTACLE AVOIDANCE VALIDATION RESULTS"
echo "=========================================="

SCORE_PERCENT=$(echo "scale=1; $PASS_COUNT * 100 / $TOTAL_TESTS" | bc)
echo "Total Tests: $TOTAL_TESTS"
echo "Passed: $PASS_COUNT"
echo "Validation Score: ${SCORE_PERCENT}%"

if (( $(echo "$SCORE_PERCENT >= 85" | bc -l) )); then
    echo -e "\n${GREEN}🏆 HUMANOID OBSTACLE AVOIDANCE: VALIDATED SUCCESS${NC}"
    echo "✅ Your avoidance system is suitable for humanoid bipedal navigation!"
    echo "✅ Meets safety requirements for ceiling, torso, and footstep protection"
    echo "✅ System ablates wheeled-robot assumptions, implements bipedal needs"
else
    echo -e "\n${RED}⚠️  VALIDATION: NEEDS IMPROVEMENT${NC}"
    echo "Some parameters outside safety limits for bipedal navigation"
    echo "Review failed tests and reconfigure safety parameters"
fi

echo "
🎓 EDUCATIONAL ACHIEVEMENT:"echo "Students understand humanoid-specific obstacle requirements:"
echo "- 3D collision prediction beyond 2D wheeled models"
echo "- Head clearance protection for ceiling environments"
echo "- Torso rotation swept volumes during navigation"
echo "- Dynamic footstep projection with gait phase awareness"
echo "✓ Ready for systematic exterior input embrace ✅"