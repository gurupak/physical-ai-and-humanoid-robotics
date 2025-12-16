# Roll/Pitch Compensation for Humanoid Navigation

Implement attitude stabilization, gravity compensation, and balance recovery during navigation to maintain directional accuracy despite bipedal locomotion disturbances and terrain variations.

## Quick Setup: Roll/Pitch Stabilization (5 minutes)

### 1. IMU-Based Attitude Compensation

```python title="attitude_compensation_controller.py - Complete roll/pitch compensation system"
#!/usr/bin/env python3
"""
Attitude Compensation Controller - Humanoid Balance Correction\nEducational implementation with real-time gravity compensation and navigation direction correction
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, JointState
from geometry_msgs.msg import Twist, Vector3, Quaternion
from geometry_msgs.msg import TransformStamped
import tf2_ros
import numpy as np
from scipy.spatial.transform import Rotation
from dataclasses import dataclass
from typing import List, Tuple
import time

@dataclass
class AttitudeMeasurement:
    """Complete attitude state with compensation calculations"""
    roll_deg: float
    pitch_deg: float
    yaw_deg: float
    roll_vel_deg: float
    pitch_vel_deg: float
    yaw_vel_deg: float
    gravity_vector: Tuple[float, float, float]  # 3D gravity direction

    # Navigation correction values
    heading_correction_rad: float
    velocity_correction_mps: float
    position_corrector_meters: float

    # Validation flags
    balance_stable: bool
    compensation_active: bool

    timestamp: float

class HumanoidAttitudeCompensator(Node):
    """Roll/pitch compensation with systematic measurement validation"""

    def __init__(self):
        super().__init__('humanoid_attitude_compensator')

        # Publishers for educational visualization
        self.attitude_pub = self.create_publisher(
            Vector3, '/humanoid/attitude/current', 10
        )
        self.correction_pub = self.create_publisher(
            Twist, '/humanoid/navigation/corrections', 10
        )
        self.balance_pub = self.create_publisher(
            Vector3, '/humanoid/balance/vector', 10
        )
        self.stability_pub = self.create_publisher(
            TransformStamped, '/humanoid/stability/transform', 10
        )

        # Multi-sensor IMU integration
        self.create_subscription(
            Imu, '/humanoid/sensors/imu/torso',
            self.torso_imu_callback, 10
        )
        self.create_subscription(
            Imu, '/humanoid/sensors/imu/base',
            self.base_imu_callback, 10
        )
        self.create_subscription(
            JointState, '/humanoid/joints/state',
            self.joint_state_callback, 10
        )

        # Configuration for systematic measurement
        self.balance_constraints = {
            'max_roll_deg': 5.0,         # ±5° roll limit for navigation accuracy
            'max_pitch_deg': 8.0,        # ±8° pitch limit during walking
            'max_roll_rate': 10.0,       # 10°/s roll rate limit
            'max_pitch_rate': 15.0,      # 15°/s pitch rate limit
            'gravity_compensation_gain': 0.98,  # 98% gravity compensation
            'heading_correction_limit': 0.1,    # ±0.1rad (±6°) max navigation correction
            'velocity_correction_limit': 0.2,   # ±0.2m/s max velocity correction
            'balance_success_threshold': 0.85   # 85% uptime for balanceteadiness
        }

        # Measurement tracking for student validation
        self.attitude_history = {
            'roll_measurements': [],
            'pitch_measurements': [],
            'yaw_measurements': [],
            'balance_stability': [],
            'correction_applied': [],
            'measurement_timestamps': []
        }

        # Student performance metrics
        self.performance_metrics = {
            'attitude_stability_score': 0.0,
            'correction_effectiveness': 0.0,
            'balance_uptime_seconds': 0.0,
            'measurements_validated': 0,
            'achievement_certified': False
        }

        self.get_logger().info("🧭 Attitude Compensation Controller: Systematic Measurement")
        self.get_logger().info("Roll/pitch correction maintaining navigation during humanoid locomotion")

    def calculate_attitude_from_imu(self, torso_imu: Imu, base_imu: Imu) -> AttitudeMeasurement:
        """
        EDUCATIONAL ALGORITHM: Calculate attitude with gravity compensation
        Integrates dual IMU data for accurate attitude with walking compensation

        Learning target: Correct navigation direction despite ±8° pitch during walking
        """

        # Extract quaternion orientations
        torso_q = torso_imu.orientation
        base_q = base_imu.orientation

        # Convert to rotation matrices
        torso_rotation = Rotation.from_quat([
            torso_q.x, torso_q.y, torso_q.z, torso_q.w
        ])
        base_rotation = Rotation.from_quat([
            base_q.x, base_q.y, base_q.z, base_q.w
        ])

        # Calculate attitude angles (Euler angles)
        torso_euler = torso_rotation.as_euler('xyz', degrees=True)
        base_euler = base_rotation.as_euler('xyz', degrees=True)

        # Apply fusion for walking movement compensation
        # Higher weight to base IMU for ground reference during walking
        comp_weight = 0.7 if self.is_walking_active() else 0.5

        roll_deg = torso_euler[0] * (1 - comp_weight) + base_euler[0] * comp_weight
        pitch_deg = torso_euler[1] * (1 - comp_weight) + base_euler[1] * comp_weight
        yaw_deg = torso_euler[2] * comp_weight + base_euler[2] * (1 - comp_weight)

        # Angular velocity compensation
        torso_vel = torso_imu.angular_velocity
        base_vel = base_imu.angular_velocity

        roll_vel = (torso_vel.x + base_vel.x) / 2.0
        pitch_vel = (torso_vel.y + base_vel.y) / 2.0
        yaw_vel = (torso_vel.z + base_vel.z) / 2.0

        # Calculate gravity vector for balance analysis
        gravity_vector = self.calculate_compensated_gravity(torso_rotation, base_rotation)

        # Calculate navigation corrections
        heading_correction = self.calculate_heading_correction(pitch_deg, roll_deg)
        velocity_correction = self.calculate_velocity_correction(torso_vel, base_vel)
        position_correction = self.calculate_position_correction(gravity_vector)

        return AttitudeMeasurement(
            roll_deg=roll_deg,
            pitch_deg=pitch_deg,
            yaw_deg=yaw_deg,
            roll_vel_deg=np.degrees(roll_vel),
            pitch_vel_deg=np.degrees(pitch_vel),
            yaw_vel_deg=np.degrees(yaw_vel),
            gravity_vector=gravity_vector,
            heading_correction_rad=heading_correction,
            velocity_correction_mps=velocity_correction,
            position_corrector_meters=position_correction,
            balance_stable=self.assess_balance_stability(torso_imu, base_imu),
            compensation_active=True,
            timestamp=time.time()
        )

    def assess_balance_stability(self, torso_imu: Imu, base_imu: Imu) -> bool:
        """
        Critical balance preparation before navigation correction
        Determines if attitude corrections are safe.
        """

        # Extract raw accelerations
        torso_acc = np.array([torso_imu.linear_acceleration.x,
                            torso_imu.linear_acceleration.y,
                          torso_imu.linear_acceleration.z])
        base_acc = np.array([base_imu.linear_acceleration.x,
                           base_imu.linear_acceleration.y,
                          base_imu.linear_acceleration.z])

        # Quick stability assessment
        # Check if robot is falling (not walking normally)
        gravity_check = np.abs(torso_acc[2] - base_acc[2])  # Vertical acceleration difference
        horizontal_rms = np.sqrt(torso_acc[0]**2 + torso_acc[1]**2)

        # Determine balance status based on acceleration patterns
        # Falling: Large vertical difference and extreme horizontal components
        balance_stable = (gravity_check < 0.5 * 9.81) and (horizontal_rms < 2.0)

        return balance_stable

    def calculate_heading_correction(self, pitch_deg: float, roll_deg: float) -> float:
        """
        Compute navigation heading correction due to attitude changes
        Pitch affects forward component, roll affects lateral component
        """

        # Convert to radians
        pitch_rad = np.radians(pitch_deg)
  roll_rad = np.radians(roll_deg)

        # Navigation direction is combination of attitude correctors
        heading_correction_yaw = np.arctan2(
      np.sin(roll_rad),  # Lateral vs forward priority
            np.cos(pitch_rad)
        )

        # Apply correction limitation to prevent navigation instability'
        max_correction = self.balance_constraints['heading_correction_limit']
        return np.clip(heading_correction_yaw, -max_correction, max_correction)

    def calculate_velocity_correction(self, torso_vel: Vector3, base_vel: Vector3) -> float:
  """
        Apply counter velocity when attitude affects platform velocity
        Prevents navigation drift during torso sway
        """

        # Calculate required compensation to maintain platform velocity
        # During standing still, this compensates for torso sway
        compensation_x = 0.02 * (base_vel.z - torso_vel.z)  # Forward component
        compensation_y = 0.01 * (base_vel.x - torso_vel.x)  # Lateral component

        # Combine into velocity magnitude
        total_compensation = np.sqrt(compensation_x**2 + compensation_y**2)

       # Limit compensation to prevent navigation oscillation
        max_correction = self.balance_constraints['velocity_correction_limit']
        return np.clip(total_compensation, -max_correction, max_correction)</reasoning>continue