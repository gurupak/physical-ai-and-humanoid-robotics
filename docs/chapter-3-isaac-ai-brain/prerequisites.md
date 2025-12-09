# Prerequisites - Chapter 3: The AI-Robot Brain

## Hardware Requirements

### Minimum Configuration
- **GPU**: NVIDIA RTX 3060 (8GB VRAM)
- **RAM**: 32GB DDR4
- **Storage**: 50GB available for Isaac Sim + 10GB for ROS2 workspace
- **CPU**: Quad-core (8 threads) @ 2.8GHz
- **OS**: Ubuntu 22.04 LTS or 20.04 LTS

### Recommended Configuration
- **GPU**: NVIDIA RTX 4090 (24GB VRAM) or RTX 3090
- **RAM**: 64GB DDR4
- **Storage**: 100GB SSD for better I/O performance
- **CPU**: 8-core (16 threads) @ 3.5GHz
- **OS**: Ubuntu 22.04 LTS with recent kernel

## Software Prerequisites

### Essential Dependencies
```bash
# Ubuntu packages
sudo apt update && sudo apt install -y \
    build-essential \
    cmake \
    git \
    python3.8+ \
    curl \
    wget \
    unzip \
    tar

# NVIDIA drivers
sudo apt install -y nvidia-driver-535  # or latest
sudo apt install -y nvidia-cuda-toolkit
```

### ROS2 Installation
```bash
# Install ROS2 Humble Hawksbill
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update
sudo apt install -y ros-humble-desktop

# Source ROS2
source /opt/ros/humble/setup.bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
```

### CUDA Installation
```bash
# Minimum CUDA 11.8 required for Isaac Sim
wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2204/x86_64/cuda-keyring_1.1-1_all.deb
sudo dpkg -i cuda-keyring_1.1-1_all.deb
sudo apt update
sudo apt install -y cuda-12-1 cuda-toolkit-12-1
```

## Knowledge Prerequisites

### Required Expertise
- **ROS2 Fundamentals**: Node, topic, service concepts
- **Basic CV/SLAM**: Understand visual odometry principles
- **Linux Proficiency**: command-line skills
- **Python/C++**: Basic programming knowledge

### Helpful Background
- Computer graphics fundamentals
- Robotic navigation concepts
- Machine learning basics
- Experience with other simulators (Gazebo, Unity)

## Key Checks Before Starting

### Verification Script
```bash
#!/bin/bash
# Run this script to verify your setup

echo "=== Prerequisites Check ==="

# Check GPU
if nvidia-smi > /dev/null 2>&1; then
    GPU_NAME=$(nvidia-smi --query-gpu=name --format=csv,noheader,nounits | head -n1)
    GPU_VRAM=$(nvidia-smi --query-gpu=memory.total --format=csv,noheader,nounits | head -n1)
    echo "✓ GPU: $GPU_NAME with ${GPU_VRAM}MB VRAM"

    # Check VRAM minimum
    if [ "$GPU_VRAM" -ge 8000 ]; then
        echo "✓ VRAM meets minimum (8GB) requirement"
    else
        echo "✗ VRAM below minimum (need 8GB+, have $GPU_VRAM MB)"
    fi
else
    echo "✗ NVIDIA GPU not detected"
fi

# Check RAM
RAM_GB=$(free -g --si | awk '/^Mem:/ {print $2}')
echo "   RAM: ${RAM_GB}GB (32GB recommended, 64GB optimal)"

# Check OS
OS_INFO=$(lsb_release -d 2>/dev/null | cut -f2)
echo "   OS: $OS_INFO"

# Check ROS2
if command -v ros2 > /dev/null 2>&1; then
    ROS2_VERSION=$(ros2 -v 2>&1 | grep humble || echo "NOT HUMBLE")
    echo "✓ ROS2 installed $ROS2_VERSION"
else
    echo "✗ ROS2 not installed"
fi

# Check CUDA
if command -v nvcc > /dev/null 2>&1; then
    CUDA_VERSION=$(nvcc --version | grep release | awk '{print $6}')
    echo "✓ CUDA $CUDA_VERSION installed"
else
    echo "✗ CUDA not installed"
fi

echo "=========================="
```



## System Compatibility Notes

### Intel vs AMD Systems
- **Intel**: Full support, verified with 12th Gen+ CPUs
- **AMD**: Full support, verified with Ryzen 5000+ series

### Cloud Options If Local GPU Unavailable
- **AWS EC2 G4dn.xlarge**: RTX T4 GPU (16GB VRAM)
- **GCP A2 VM**: V100 or A100 GPU options
- **Azure NC Series**: GPU-enabled virtual machines

### Fallback Paths
- **CPU-only Simulation**: Limited functionality, 5-10 FPS vs 60+ FPS with GPU
- **Remote Rendering**: Cloud workstation for heavy rendering tasks
- **Community Edition**: Limited but functional version of Isaac Sim

## Pre-Chapter Setup Tasks

1. [ ] Verify GPU meets minimum requirements (RTX 3060+)
2. [ ] Install Ubuntu 22.04 LTS with NVIDIA drivers
3. [ ] Install CUDA 11.8+ toolkit
4. [ ] Install ROS2 Humble Hawksbill
5. [ ] Download and test Isaac Sim (optional preview)
6. [ ] Run the verification script above
7. [ ] Join NVIDIA Developer program for access to resources

---

**All prerequisites met?** Proceed to [learning objectives](./learning-objectives.md)

**System issues?** Check [Common Errors Guide](./common-errors.md)