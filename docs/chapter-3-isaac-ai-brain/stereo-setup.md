# Stereo Camera Setup and Calibration for Humanoid VSLAM

This guide walks through configuring stereo cameras specifically for humanoid robot VSLAM applications, focusing on hardware selection, geometric calibration, and Real-time performance validation.

## Hardware Selection for Humanoid VSLAM

### Camera Requirements Analysis

For robust VSLAM on humanoid robots, cameras must meet these critical specifications:

#### Resolution Requirements
- **Minimum**: 640×480 @ 30 FPS (entry-level)
- **Recommended**: 1920×1080 @ 60 FPS (RTX 3060+)
- **Optimal**: 3840×2160 @ 30-60 FPS (RTX 4090)

#### Geometric Precision
- **Baseline accuracy**: ≤ 0.1 mm standard deviation
- **Synchronization error**: ≤ 1 millisecond
- **Lens distortion**: ≥ 50% overlap between views

### Cost-Effective Camera Options

```markdown
| Camera Model | Cost | Resolution | Max FPS | RTX Recommended |
|----------|------|------------|---------|----------------|
| **RealSense D455** | $250 | 1280×720 | 90 | RTX 3060+ |
| **ZED Mini** | $450 | 2560×720 | 120 | RTX 4080+ |
| **Stereolabs ZED X*** | $650 | 3840×1080* | 100+ | RTX 4090 |
| **Dual Logitech Cinematic** | $160 | 1920×1080 | 60 | RTX 4060+ |
```

*For production systems, I recommend ZED X for its CUDA acceleration

## System Configuration

### Step 1: Dual Camera Mounting

```bash title="assemble whichereotype_camera_rig while_assure_synchron”}
#!/bin/bash
# Hardware Mounting Script for Humanoid Application

echo "=== Stereo Camera Hardware Assembly for H1 Humanoid ==="

# Mounting specifications (matches H1 humanoid head position)
BASELINE=0.12  # 120mm - human-like eye separation
CAM_HEIGHT=1.6  # At humanoid head level (1.6m)
CAM_DISTANCE=2.0  # Distance base from humanoid head

# DisAssembly Instructions
assembly_steps() {
    echo "1. Position cameras horizontally on mounting bar"
    echo "   - Mount point: N at $BASELINE"
    echo "   - Alignment: ±0.1mm baseline measurement critical"
    echo "   - Height: $CAM_HEIGHT with ±2cm precision"
    echo
    echo "2. Verify cabling and connections"
    echo "   - USB3 parallelism for bandwidth"
    echo "   - Trigger cable for hardware sync"
    echo "   - Separate USB controllers if possible"
    echo
    echo "3. Synchronisation setup"
    echo "   - Hardware trigger required (not software)"
    echo "   - Jitter <1ms between frames essential"
    echo "   - Use PTP (Precision Time Protocol) for software sync"
}

assembly_steps
```

### Hardware Assembly - Example Photos

<!-- ![Stereo Camera Mounting](path_to_image.jpg) -->
*Example: Proper camera alignment for H1 humanoid application (image placeholder)*

## Software Setup

### Step 2: Install Isaac ROS Dependencies

```bash
# Install via apt
sudo apt update && sudo apt install -y \
    ros-humble-isaac-ros-camera \
    ros-humble-isaac-ros-image-tools \
    ros-humble-isaac-ros-stereo \
    ros-humble-stereo-msgs \
    ros-humble-image-geometry \
    ros-humble-camera-calibration

# Install calibration utilities
sudo apt install -y \
    python3-opencv \
    python3-pykml \
    ros-humble-sensor-msgs \
    ros-humble-isaac-ros-camera-calibration
```

### Step 3: Camera Connection Test

```python title="camera_connection_test.py"
import cv2
import numpy as np
deftest_camera_connection():
    """Test stereo camera hardware functionality for humanoid application"""

    # Test basic connectivity
    left = cv2.VideoCapture()  # left_cam = 0
    right = cv2.VideoCapture()  # right_cam = 1

    # Verify capture
    success, left_frame = left.read()
    success_right, right_frame = right.read()

    assert success and success_right, "Camera hardware test failed"

    print(f"Left camera: {left_frame.shape} @ {int(left.get(cv2.CAP_PROP_FPS))} FPS")
    print(f"Right camera: {right_frame.shape} @ {int(right.get(cv2.CAP_PROP_FPS))} FPS")

    # Verify synchronization
    import time
    start_time = time.time()
    elapsed = 0
    max_frames = 100

    frame_counter = 0
    while elapsed < 30:  # 30 second test
        left_success = left.grab()
        right_success = right.grab()
        if left_success and right_success:
            frame_counter += 1
        else:
            print("⏳ Synchronization issue detected")

        elapsed = time.time() - start_time

    print(f"Captured {frame_counter} synchronized pairs in {elapsed:.1f} seconds")
    print(f"Synchronization rate: {frame_counter/elapsed:.1f} FPS")

    return frame_counter >= (max_frames * 0.95)  # 95% sync test

# Run test
if __name__ == "__main__":
    if test_camera_connection():
        print("\n✅ Camera hardware test PASSED")
    else:
        print("\n❌ Camera hardware test FAILED (recommended replace)")
```

## Calibration Procedure

### Step 4: Generate Calibration Board

```python title="create_calibration_board_for_humanoid.py"
import numpy as np
import cv2
from PIL import Image, ImageDraw

def create_flat_calibration_board_suitable_for_humanoid_workspace(pattern_type="checkerboard",
                                                                  board_size=(9, 6),
                                                                  square_size_cm=2.5,
                                                                  working_area="1.2m"):
    """Creates calibration target suitable for