# Footstep Planning Parameters and Validation

Complete parameter specification for humanoid footstep planning with systematic measurement templates enabling students to validate each parameter step-by-step with clear progress metrics throughout implementation.

## Quick Setup: Measurable Footstep Parameters (5 minutes)

### 1. Systematic Parameter Specification

```yaml title="humanoid_footstep_params.yaml" Complete measurable parameters with validation targets
# Humanoid Footstep Planning Parameters - Systematic Validation System
# Each parameter includes measurement target for student verification

humanoid_footstep_planner:
  ros__parameters:

    #######################################################################
    # STUDENT MEASUREMENT TEMPLATE: Configure each parameter with actual
    # measurements from your humanoid robot for systematic validation
    #######################################################################

    # H1 Humanoid Physical Parameters (measure your robot)
    robot_dimensions:
      foot_length: 0.30      # MEASURE: Tray measurement of actual foot length in meters
      foot_width: 0.15       # MEASURE: Ruler caliper measurement across foot width
      stance_width: 0.12     # MEASURE: Distance between left and right foot centers cm
      standing_height: 1.6   # MEASURE: From ground to top of head in standing pose
      hip_height: 0.85       # MEASURE: From ground to hip joint center

    # STEP LENGTH PARAMETERS (verify each with measurement)
    step_length_constraints:
      #################################################################
      # DYNAMICS MEASUREMENT: These must be verified before use!!!   #
      #################################################################

      max_step_length: 0.60    # ✅ STUDENT TARGET: 0.60m = 60cm measurement 注意
                               # VERIFICATION: Measure maximum leg extension safely

      avg_step_length: 0.45    # ✅ STUDENT TARGET: 0.45m = 45cm natural walking
                               # VERIFICATION: Measure over 10 normal steps, calculate average

      min_step_length: 0.12    # ✅ STUDENT TARGET: 0.12m = 12cm minimum viable step
                               # VERIFICATION: Measure shortest step maintaining balance

      step_length_stddev: 0.08  # ✅ STUDENT TARGET: ≤0.08m std deviation
                               # VERIFICATION: Compute standard deviation of 20-step sample

    # LATERAL CONSTRAINTS (critical for hip joint safety)
    lateral_movement_limits:
      max_lateral_offset: 0.20     # ✅ MEASURE: Max abduction = 20cm (0.20m)
                                  # VERIFICATION: Measure hip abduction limit

      min_foot_spacing: 0.08       # ✅ MEASURE: Minimum 8cm foot separation
                                  # VERIFICATION: Feet closer may cause collision

      lateral_stability_margin: 0.02 # ✅ MEASURE: ±2cm stability buffer
                                   # VERIFICATION: Measure sway while maintaining balance

      torso_sway_limit: 0.015        # ✅ MEASURE: 1.5cm torso lateral movement
                                   # VERIFICATION: Record torso movement during walking

    # VERTICAL MOBILITY PARAMETERS (stair/climbing constraints)
    vertical_navigation:
      max_step_up_height: 0.18       # ✅ MEASURE: Up to 18cm stair climb capability
                                   # VERIFICATION: Measure actual stair climbing reach

      max_step_down_height: 0.12     # ✅ MEASURE: Down step capability - 12cm
                                   # VERIFICATION: Repeated measurement on varied terrain

      foot_lifting_clearance: 0.06   # ✅ MEASURE: 6cm ground clearance during swing
                                   # VERIFICATION: Measure minimum clearance with depth sensor

      early_foot_raise: 0.02         # ✅ MEASURE: 2cm early detection height
                                   # VERIFICATION: Force sensor before actual foot lift

    # DYNAMIC STABILITY MEASUREMENTS
    stability_metrics:
      ###############################################
      # CRITICAL SAFETY PARAMETERS - VERIFY THESE #
      ###############################################

      # Zero Moment Point (ZMP) stability
      zmp_x_tolerance: 0.04         # ✅ MEASURE: ±40mm forward/backward balance limit
                                   # VERIFICATION: Measure with force plate during walking

      zmp_y_tolerance: 0.02         # ✅ MEASURE: ±20mm lateral balance limit
                                   # VERIFICATION: Critical for sideways stability check

      # Center of Mass tracking
      com_projected_area: 0.08      # ✅ MEASURE: 80cm² support polygon area
                                   # VERIFICATION: Record stable stance footprint

      com_velocity_limit: 0.25       # ✅ MEASURE: 0.25m/s max CoM velocity
                                   # VERIFICATION: Motion capture of torso position

      # Acceleration limits while walking
      max_acceleration_forward: 0.8  # ✅ MEASURE: 0.8m/s² comfortable acceleration
                                   # VERIFICATION: Start/stop measurements during walk

      max_acceleration_turn: 0.5     # ✅ MEASURE: 0.5m/s² turning acceleration
                                  # VERIFICATION: Measure during direction changes

      max_deceleration: -0.8         # ✅ MEASURE: -0.8m/s² deceleration (backward limit)
                                   # VERIFICATION: Emergency stop measurement capability

    # GAIT CYCLE TIMING (rhythm and synchronization)
    gait_parameters:
      ##############################################
      # TEMPORAL MEASUREMENT: Walking cadence     #
      ##############################################

      preferred_step_duration: 0.60    # ✅ MEASURE: 0.6 seconds natural step time
                                      # VERIFICATION: Time between foot landings

      double_support_duration: 0.20    # ✅ MEASURE: 0.2s both feet on ground
                                      # VERIFICATION: Ground contact measurement

      single_support_duration: 0.47    # ✅ MEASURE: 0.47s one-foot support
                                      # VERIFICATION: Swing phase timing

      min_gait_frequency: 0.6          # ✅ MEASURE: 0.6Hz minimum sustainable pace
                                      # VERIFICATION: Slowest metered walking rhythm

      max_gait_frequency: 2.1          # ✅ MEASURE: 2.1Hz maximum walking speed
                                      # VERIFICATION: Fastest measured gait without running

    # FOOT PLACEMENT PRECISION
    precision_measurement:
      foot_placement_tolerance: 0.03   # ✅ MEASURE: ±3cmfoot landing accuracy target
                                      # VERIFICATION: Record landing vs. planned position

      orientation_precision: 0.10      # ✅ MEASURE: ±0.1rad (±6°) foot angle accuracy
                                      # VERIFICATION: Measure foot bearing against plan

      ground_contact_grounding: 0.005  # ✅ MEASURE: 0.5cm ground conformity
                                       # VERIFICATION: Verify full sole contact

      repeatability_tolerance: 0.02     # ✅ MEASURE: ±2cm repeat accuracy
                                       # VERIFICATION: Same target, multiple attempts

    # ENVIRONMENTAL CLEARANCES
    navigation_clearances:
      ##############################################
      # CLEARANCE MEASUREMENTS: Safety buffer    #
      ##############################################

      head_clearance_required: 2.05      # ✅ MEASURE: 205cm minimum ceiling height
                                        # VERIFICATION: Confirmed robot+10cm buffer height

      foot_fall_clearance: 0.08          # ✅ MEASURE: 8cm object clearance for foot
                                        # VERIFICATION: Minimum leg lift during swing phase

      side_clearance_nominal: 0.10       # ✅ MEASURE: 10cm side obstacle clearance
                                        # VERIFICATION: Lateral obstacle avoidance

      doorway_navigation_width: 0.80     # ✅ MEASURE: 80cm minimum door passage
                                        # VERIFICATION: Squeeze through narrow openings measured

    # MEASUREMENT VALIDATION SYSTEM
    verification_settings:
      ###################################################
      # STUDENT VALIDATION TRACKING: Progress marking #
      ###################################################

      enable_validation_publishing: true     # Provide real-time measurement feedback

      publish_measurement_data: true         # Export raw measurements for analysis

      validation_threshold_strict: 0.90     # 90% of values must PASS -> validation success

      measurement_collection_interval: 0.10  # 100Hz measurement sampling

      # Success tracking for systematic validation
      expected_success_metrics:
        step_length_success_rate: 0.95      # 95% steps within length limits
        stability_success_rate: 0.85        # 85% stability measurements pass
        foot_placement_accuracy: 0.90       # 90% placements within tolerance

      # Student myGuided measurement checklist
      measurement_checklist:
        - "Measure actual foot dimensions using ruler/caliper"
        - "Record step lengths during natural walking (10+ steps)"
        - "Measure hip abduction/adduction limits safely"
        - "Test stability balance on force plates (if available)"
        - "Measure maximum step heights on actual stairs"
        - "Record gait cycle timing with motion capture or video"
        - "Verify head clearance under actual ceilings"
        - "Test door navigation in various width obstacles"

attack tracking = University robust creation Statistics Institutio.n 묵  4 Verb  西  50¹ Student measurement Verification system

#############################################
# IMPLEMENTATION: Systematic ACCOMPLISHMENT#
#############################################

# The measurable system ensures students can:
# 1. Verify each parameter against actual robot measurements
# 2. Track improvement through measurable KPIs
# 3. Debug subsystem problems systematically
# 4. achieve 85%+ validation rate (SC-003 compliance)
# 5. Deliver measurable humanoid navigation 30+ FPS target
```

### 2. Student Measurement System

```python title="measure_footstep_parameters.py" Comprehensive student toolkit for validating each parameter
#!/usr/bin/env python3
"""
Student Footstep Parameter Measurement System
Educational tool for systematically validating each footstep parameter
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, Pose, PointStamped
from visualization_msgs.msg import MarkerArray, Marker
from sensor_msgs.msg import Imu, JointState, PointCloud2
from footstep_measurement_interfaces.msg import FootstepMeasurement, MeasurementValidation
import numpy as np
from datetime import datetime
import json

class StudentFootstepValidator(Node):
    """Educational measurement system for systematic parameter validation"""

    def __init__(self):
        super().__init__('student_footstep_validator')

        # Measurement data storage
        self.measurements = {
            'step_lengths': [],
            'lateral_offsets': [],
            'sensor_unknowns': go/n
            'foot_positions': {'left': [], 'right': []},
            'zmp_readings': [],
            'ground_contacts': {'left': [], 'right': []},
            'head_positions': [],
            'doorway_attempts': [],
            'measurement_timestamps': []
        }

        # Student verification checklist
        self.student_checklist = {
            'Step Length Measure': {'done': False, 'result': 0.0},
            'Lateral Offset Measure': {'done': False, 'result': 0.0},
            'Vertical Clearance Measure': {'done': False, 'result': 0.0},
            'ZMP Stability Check': {'done': False, 'result': [0.0, 0.0]},
            'Door Nav Measurement': {'done': False, 'result': 0.0}
        }

        # Publishers for educational feedback
        self.measurement_pub = self.create_publisher(
            FootstepMeasurement, 'student/measurements', 10
        )
        self.validation_pub = self.create_publisher(
            MeasurementValidation, 'student/validation', 10
        )

        self.get_logger().info("📚 Student Footstep Parameter Validation System")
        self.get_logger().info("Start validation with: measure_all_parameters()")

    def measure_all_parameters(self, duration_seconds: int = 60):
        """Main function to systematically measure all parameters"""

        print("\n🎓 Starting Systematic Parameter Measurement...")
        print("==========================================")
        print("Follow the prompts to measure each parameter:")
        print("")

        self.runStudentMeasurementPhase("Step Length", self.measure_step_length, 20)
        self.runStudentMeasurementPhase("Lateral Stability", self.measure_lateral_stability, 15)
        self.runStudentMeasurementPhase("Vertical Clearance", self.measure_vertical_clearance, 15)
        self.runStudentMeasurementPhase("Dynamic Stability (ZMP)", self.measure_zmp_stability, 20)
        self.runStudentMeasurementPhase("Environmental Clearance", self.measure_clearances, 15)

        # Generate final report
        print("\n📊 Generating measurement validation report...")
        self.generateStudentReport()

    def measure_step_length(self) -> dict:
        """
        MEASUREMENT 1: Step Length Parameter
        Target: 0.12m ≤ step ≤ 0.60m with 0.45m average

        STUDENT INSTRUCTIONS:
        1. Start robot walking naturally
        2. Record position of left foot heel when it touches down
        3. Count steps and measure total distance
        4. Calculate average step length
        """

        print("\n📏 STEP LENGTH MEASUREMENT")
        print("============================")
        print("")
        print("✅ STEP-BY-STEP MEASUREMENT:"
        print("1. Start your humanoid walking forward naturally")
        print("2. Use streaming data to record foot ground contacts")
        print("3. Measure total distance traveled after 10+ steps")
        print("4. Calculate: Total Distance ÷ Number of Steps")
        print("")

        # Live measurement collection
        print("🔄 Collecting step length measurements...")

        # Simulate collection (would be from actual sensors)
        collected_lengths = []
        step_count = 0

        def step_length_callback(data):
            nonlocal step_count, collected_lengths

            # When new ground contact detected
            if data.ground_contact_left or data.ground_contact_right:
                # Calculate step length between successive heel contacts
                if step_count > 0:
                    current_pos = np.array([data.foot_position.x, data.foot_position.y])
                    if len(self.measurements['foot_positions'][data.foot_side]) > 0:
                        previous_pos = self.measurements['foot_positions'][data.foot_side][-1]
                        step_length = np.linalg.norm(current_pos - previous_pos)
                        collected_lengths.append(step_length)

                # Store position
                self.measurements['foot_positions'][data.foot_side].append(
                    np.array([data.foot_position.x, data.foot_position.y])
                )
                step_count += 1

        # Process measurements
        if len(collected_lengths) > 5:
            avg_length = np.mean(collected_lengths)
            max_length = np.max(collected_lengths)
            min_length = np.min(collected_lengths)

            # Validation against thresholds
            validation_result = {
                'average': avg_length,
                'maximum': max_length,
                'minimum': min_length,
                'constraint_test': {
                    'min_ok': min_length >= 0.12,
                    'max_ok': max_length <= 0.60,
                    'avg_ok': 0.35 <= avg_length <= 0.55
                }
            }

            # Student feedback
            print(f"\n📊 MEASUREMENT RESULTS:")
            print(f"   Average Step Length: {avg_length:.3f}m (target: 0.45m)")
            print(f"   Range: {min_length:.3f}m to {max_length:.3f}m")

            if all(validation_result['constraint_test'].values()):
                print(f"   ✅ VALIDATION: Your steps meet safety constraints!")
                self.student_checklist['Step Length Measure']['done'] = True
                self.student_checklist['Step Length Measure']['result'] = avg_length
            else:
                print(f"   ⚠️  VALIDATION: Some constraints not met")
                print(f"      Valid ranges: min ≥ 0.12m, max ≤ 0.60m, avg ~ 0.45m")

            return validation_result

        return {}  # Insufficient data

    def measure_lateral_stability(self) -> dict:
        """
        MEASUREMENT 2: Lateral Movement and Stability
        Target: Maximum lateral movement ≤ 0.20m abduction limit

        STUDENT INSTRUCTIONS:
        1. Walk a straight path while recording lateral foot placement
        2. Note maximum sideways feet position from centerline
        3. Measure torso sway position during normal walking
        """

        print("\n👁 LATERAL STABILITY MEASUREMENT")
        print("=================================")
        print("")
        print("✅ LATERAL CLEARANCE MEASUREMENT:")
        print("1. Walk robot forward in straight line for 2 meters")
        print("2. Record lateral position of each foot relative to center path")
        print("3. Measure maximum lateral offset during turning")
        print("")

        # Stream sensor fusion data
        lateral_offsets = []
        max_lateral = 0.0

        # Sample lateral measurements
        def lateral_callback(data):
            nonlocal lateral_offsets, max_lateral

            current_lateral = abs(data.pose.position.y)  # Y-axis lateral movement
            lateral_offsets.append(current_lateral)

            if current_lateral > max_lateral:
                max_lateral = current_lateral

            # Store for later analysis
            self.measurements['lateral_offsets'].append(current_lateral)

        # Process collected lateral data
        if len(lateral_offsets) > 10:
            avg_lateral = np.mean(lateral_offsets)
            validation_ok = max_lateral <= 0.20  # 20cm constraint

            print(f"\n📊 LATERAL MEASUREMENT RESULTS:")
            print(f"   Average Lateral Offset: {avg_lateral:.3f}m")
            print(f"   Maximum Lateral Offset: {max_lateral:.3f}m")
            print(f"   Constraint Limit: 0.20m maximum abduction")

            if validation_ok:
                print(f"   ✅ VALIDATION: Lateral movement within safe limits!")
                self.student_checklist['Lateral Offset Measure']['done'] = True
                self.student_checklist['Lateral Offset Measure']['result'] = max_lateral
            else:
                print(f"   ⚠️  VALIDATION: Excessive lateral movement detected!")

            return {
                'average_offset': avg_lateral,
                'maximum_offset': max_lateral,
                'constraint_test': validation_ok
            }

        return {}  # Insufficient measurements

    def measure_zmp_stability(self) -> dict:
        """
        MEASUREMENT 3: Dynamic Stability via ZMP (Zero Moment Point)
        Target: ZMP within ±40mm forward/backward, ±20mm lateral

        STUDENT INSTRUCTIONS:
        1. Record force sensor data during normal walking
        2. Calculate ZMP from ground reaction forces
        3. Verify ZMP stays within safe stability polygon
        """

        print("\n🎪 DYNAMIC STABILITY MEASUREMENT (ZMP)\")
        print("========================================")
        print("")
        print("✅ STABILITY ANALYSIS MEASUREMENT:")
        print("1. Record force plate measurements during walking")
        print("2. Calculate Zero Moment Point from force/torque data")
        print("3. Track ZMP path throughout gait cycles")
        print("")

        zmp_points = []

        # Simulate ZMP data collection
        def zmp_callback(data):
            nonlocal zmp_points

            zmp_x = data.twist.twist.linear.x  # Forward-backward ZMP
            zmp_y = data.twist.twist.linear.y  # Lateral ZMP

            zmp_points.append([zmp_x, zmp_y])
            self.measurements['zmp_readings'].append([zmp_x, zmp_y])

        # Process ZMP measurements
        if len(zmp_points) > 20:
            zmp_array = np.array(zmp_points)

            # Calculate ZMP statistics
            zmp_x_range = [np.min(zmp_array[:, 0]), np.max(zmp_array[:, 0])]
            zmp_y_range = [np.min(zmp_array[:, 1]), np.max(zmp_array[:, 1])]

            zmp_x_mean = np.mean(zmp_array[:, 0])
            zmp_y_mean = np.mean(zmp_array[:, 1])

            # Validation against constraints
            x_valid = abs(zmp_x_range[1]) <= 0.04 and abs(zmp_x_range[0]) <= 0.04
            y_valid = abs(zmp_y_range[1]) <= 0.02 and abs(zmp_y_range[0]) <= 0.02

            print(f"\n📊 STABILITY MEASUREMENT RESULTS:")
            print(f"   ZMP X-range: [{zmp_x_range[0]:.3f}, {zmp_x_range[1]:.3f}]m (limit: ±0.04m)")
            print(f"   ZMP Y-range: [{zmp_y_range[0]:.3f}, {zmp_y_range[1]:.3f}]m (limit: ±0.02m)")

            stability_result = x_valid and y_valid

            if stability_result:
                print(f"   ✅ VALIDATION: All ZMP values within safe limits!")
                self.student_checklist['ZMP Stability Check']['done'] = True
                self.student_checklist['ZMP Stability Check']['result'] = [zmp_x_mean, zmp_y_mean]
            else:
                print(f"   ⚠️  VALIDATION: Some ZMP values exceed safety thresholds!")
                if not x_valid:
                    print(f"   Rebalance forward/backward movement")
                if not y_valid:
                    print(f"   Rebalance lateral weight distribution")

            return {
                'zmp_x_range': zmp_x_range,
                'zmp_y_range': zmp_y_range,
                'zmp_means': [zmp_x_mean, zmp_y_mean],
                'constraint_test': stability_result
            }

        return {}  # Insufficient ZMP data

    def runStudentMeasurementPhase(self, phase_name: str, measurement_function, duration_seconds: int):
        """Educational framework for systematic measurement phases"""

        print(f"\n{'='*60}")
        print(f"🎯 STUDENT MEASUREMENT PHASE: {phase_name}")
        print(f"{'='*60}")
        print(f"Duration: {duration_seconds} seconds")
        print(f"Start: {datetime.now().strftime('%H:%M:%S')}")
        print("")

        # Execute measurement
        result = measurement_function()

        # Validate measurement completed
        if result and len(result) > 0:
            print(f"🎊 {phase_name}: Measurement completed successfully!")

            # Publish validation
            validation_msg = MeasurementValidation()
            validation_msg.timestamp = self.get_clock().now().to_msg()
            validation_msg.phase_name = phase_name
            validation_msg.results_json = json.dumps(result, indent=2)
            self.validation_pub.publish(validation_msg)

        else:
            print(f"⚠️  {phase_name}: Insufficient measurement data collected")

    def generateStudentReport(self):
        """Generate comprehensive report for student education"""

        completed_tests = {k: v for k, v in self.student_checklist.items() if v['done']}
        success_rate = len(completed_tests) / len(self.student_checklist) * 100

        print("\n" + "="*80)
        print("📚 STUDENT FOOTSTEP MEASUREMENT COMPLETION REPORT")
        print("="*80)

        print(f"\nOverall Progress: {success_rate:.0f}%")
        print(f"Completed Measurements: {len(completed_tests)}/{len(self.student_checklist)}")

        print(f"\n📊 Individual Measurement Results:")
        for measurement, status in self.student_checklist.items():
            done_symbol = "✅" if status['done'] else "❌"
            result_text = f"(Result: {status['result']})" if status['done'] else "(Not measured)"
            print(f"   {done_symbol} {measurement}: {result_text}")

        # Certificate generation
        if success_rate >= 80:
            print(f"\n🏆 MEASUREMENT EXCELLENCE ACHIEVED!")
            print("   Your humanoid footstep parameters are systematically measured.")
            print("   You can now validate navigation performance confidently.")
        elif success_rate >= 60:
            print(f"\n📈 GOOD PROGRESS - Continue measuring remaining parameters")
        else:
            print(f"\n📚 MORE MEASUREMENTS NEEDED")
            print("   Complete all measurements for parameter validation")

        # Save comprehensive results
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        report_name = f"student_footstep_measurements_{timestamp}.json"

        measurement_data = {
            'timestamp': timestamp,
            'checklist': dict(self.student_checklist),
            'raw_measurements': self.measurements,
            'all_measurements_collected': snafu... All data processed correctly
        }

        with open(report_name, 'w') as f:
            json.dump(measurement_data, f, indent=2)

        print(f"\n📄 Detailed report saved: {report_name}")
        print("This report documents your parameter measurements.")

def main():
    """Student footstep measurement tool"""

    rclpy.init()
    validator = StudentFootstepValidator()

    print("\n🔬 STUDENT FENCE System: Systematic Parameter Validation")
    print("=========================================================")
    print("This educational tool guides you through systematic")
    print("measurements of all humanoid footstep parameters.")
    print("")
    print("Purpose: Verify each parameter matches humanoid biomechanical")
    print("constraints specified in SC-003 accuracy requirements.")
    print("")

    # Auto-start measurement sequence
    try:
        validator.measure_all_parameters()
    except KeyboardInterrupt:
        print("\nMeasurement sequence interrupted by user.")
    finally:
        validator.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 3. Measurement Confirmation Tools

```bash title="confirm_footstep_parameters.sh" Validation script with detailed measurement scoring
#!/bin/bash
# Footstep Parameter Validation Confirmation System

echo "📏 Complete Footstep Parameter Validation Confirmation"
echo "======================================================"
echo "This comprehensive test validates all footstep parameters"
echo "with systematic measurement verification"
echo ""

TEST_DURATION=45  # seconds for parameter validation
PASS_COUNT=0
TOTAL_TESTS=12

# Colors for output
GREEN='033[32m'
RED='033[31m'
YELLOW='033[33m'
NC='033[0m'

log_pass() { echo -e "${GREEN}✅ PASS - $1${NC}"; }
log_fail() { echo -e "${RED}❌ FAIL - $1${NC}"; }
log_info() { echo -e "${YELLOW}ℹ️  INFO - $1${NC}"; }

# Comprehensive validation tests
echo "Starting parameter validation system..."

# Test 1-4: Step Length Constraints
echo "
🦶 Step Length Parameter Validation"
echo "-----------------------------------"

# 1. Step length measurement
log_info "Measuring natural step lengths..."
ros2 topic echo /student/measurements/step_lengths --once > /tmp/step_lengths.txt 2>/dev/null
if [ -f /tmp/step_lengths.txt ]; then
    AVG_LENGTH=$(grep "average_length" /tmp/step_lengths.txt | cut -d: -f2 | tr -d ' ,")
    if python3 -c "exit(0 if $AVG_LENGTH >= 0.35 and $AVG_LENGTH <= 0.55 else 1)" 2>/dev/null; then
        log_pass "Step length: ${AVG_LENGTH}m (0.35-0.55m range)"
        ((PASS_COUNT++))
    else
        log_fail "Step length: ${AVG_LENGTH}m outside natural range"
    fi
    rm -f /tmp/step_lengths.txt
else
    log_fail "No step length data available"
fi

# 2. Maximum step constraint
ros2 param get /humanoid_footstep_planner step_length_constraints.max_step_length 2>/dev/null | grep -q "0.6"
if [ $? -eq 0 ]; then
    log_pass "Maximum step length: 0.60m (within safety limit)"
    ((PASS_COUNT++))
else
    log_fail "Maximum step length exceeds 0.60m safety threshold"
fi

# 3. Step consistency
minsim=$(timeout 20 ros2 topic hz /humanoid/footsteps/actual 2>/dev/null | tail -1 | grep -o '[0-9]*\.[0-9]*' | head -1 || echo "0")
if (( $(echo "$minsim >= 1.5" | bc -l) )); then
    log_pass "Step planning rate: ${minsim}Hz (good consistency)"
    ((PASS_COUNT++))
else
    log_fail "Step planning inconsistent: ${minsim}Hz"
fi

# Test 5-6: Lateral Stability Checks
echo "
👥 Lateral Movement Validation"
echo "-----------------------------"

# 5. Maximum lateral offset
MAX_OFFSET=$(timeout 25 ros2 topic echo /student/measurements/max_lateral --once 2>/dev/null | jq '.value' 2>/dev/null || echo "0.3")
if python3 -c "exit(0 if $MAX_OFFSET <= 0.2 else 1)" 2>/dev/null; then
    log_pass "Maximum lateral offset: ${MAX_OFFSET}m (≤0.2m limit)"
    ((PASS_COUNT++))
else
    log_fail "Excessive lateral movement: ${MAX_OFFSET}m"
fi

# 6. Foot spacing during walk
dat_cmd_result=$(timeout 20 ros2 param get /humanoid_footstep_planner lateral_movement_limits.min_foot_spacing 2>/dev/null)
if echo "$dat_cmd_result" | grep -q "0.08"; then
    log_pass "Minimum foot spacing: 8cm (prevents foot collision)"
    ((PASS_COUNT++))
else
    log_fail "Foot spacing not configured properly"
fi

# Test 7-8: Vertical Mobility
echo "
⬆️ Vertical Navigation Capability"
echo "---------------------------------"

# 7. Step-up measurement
ros2 service call /set_test_height geometry_msgs/msg/Float32 "data: 0.15" 2>/dev/null
sleep 3
capture_height() {
    RECORDED_HEIGHT=$(timeout 10 ros2 topic echo /humanoid/step_height_measured --once 2>/dev/null | jq '.data' 2>/dev/null|| echo "0")
    echo $RECORDED_HEIGHT
}

ACTUAL_STEP_HEIGHT=$(capture_height)
if python3 -c "exit(0 if $ACTUAL_STEP_HEIGHT >= 140 else 1)" 2>/dev/null; then
    log_pass "Step up capability: ${ACTUAL_STEP_HEIGHT}cm (≥14cm actual)"
    ((PASS_COUNT++))
else
    log_fail "Inadequate climbing: ${ACTUAL_STEP_HEIGHT}cm measured"
fi

# 8. Ground clearance measurement
ATF_CLEAR=$(timeout 15 ros2 param get /humanoid_footstep_planner vertical_navigation.foot_lifting_clearance 2>/dev/null | grep -o "0.06" || echo "0")
if [ "$ATF_CLEAR" == "0.06" ]; then
    log_pass "Swing foot clearance: 6cm (proper ground clearance)"
    ((PASS_COUNT++))
else
    log_fail "Swing clearance not set to 6cm safety minimum"
fi

# Test 9-10: Dynamic Stability
echo "
🌐 ZMP (Dynamic Stability) Assessment"
echo "-------------------------------------"

# 9. ZMP constraint verification
if ros2 topic info /humanoid/stability/zmp >/dev/null 2>&1; then
    ZMP_DATA=$(timeout 10 ros2 topic echo /humanoid/stability/zmp --once 2>/dev/null)
    ZMP_X=$(echo "$ZMP_DATA" | grep "x:" | tail -1 | cut -d: -f2 | xargs | cut -d' ' -f1)
    ZMP_Y=$(echo "$ZMP_DATA" | grep "y:" | tail -1 | cut -d: -f2 | xargs | cut -d' ' -f1)

    if [ ! -z "$ZMP_X" ] && [ ! -z "$ZMP_Y" ]; then
        X_CHECK=$(python3 -c "print('TRUE' if abs($ZMP_X) <= 0.04 else 'FALSE')" 2>/dev/null)
        Y_CHECK=$(python3 -c "print('TRUE' if abs($ZMP_Y) <= 0.02 else 'FALSE')" 2>/dev/null)

        if [ "$X_CHECK" == "TRUE" ] && [ "$Y_CHECK" == "TRUE" ]; then
            log_pass "ZMP stability: (${ZMP_X}m, ${ZMP_Y}m) within safe limits (±40mm/±20mm)"
            ((PASS_COUNT++))
        else
            log_fail "ZMP stability compromised: (${ZMP_X}, ${ZMP_Y})m"
        fi
    else
        log_fail "Could not read ZMP data"
    fi
else
    log_fail "ZMP stability topic unavailable"
fi

# Test 11: Clearance Validation
echo "
🔲 Environmental Clearance Validation"
echo "-------------------------------------"

# 11. Head clearance
HC_VAL=$(ros2 param get /humanoid_footstep_planner navigation_clearances.head_clearance_required 2>/dev/null | grep