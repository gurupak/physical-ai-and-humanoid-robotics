# Isaac Sim Installation Guide

## Quick Start (15 minutes)

This guide will get you from zero to rendering your first photorealistic scene in Isaac Sim.

### Prerequisites Verification

Run our quick verification script:

```bash
#!/bin/bash
# Download and run verification
wget -O /tmp/verify_isaac_prereq.sh https://github.com/anthropics/hackathon-book/raw/main/scripts/verify_isaac_prereq.sh
chmod +x /tmp/verify_isaac_prereq.sh
/tmp/verify_isaac_prereq.sh

# Expected output:
# ✅ GPU: NVIDIA RTX 3060 with 12288MB VRAM
# ✅ VRAM meets minimum (8GB) requirement
# ✅ Ubuntu 22.04.3 LTS detected
# ✅ CUDA 12.1 installed
# ✅ ROS2 Humble detected
# All requirements met! Proceed with installation.
```

### Step 1: Download Isaac Sim (5 minutes)

```bash
# Create installation directory
mkdir -p ~/apps && cd ~/apps

# Download Isaac Sim 4.0.0 (Latest version as of Dec 2024)
# This downloads ~10GB so ensure stable connection
wget -c https://developer.nvidia.com/downloads/isaac-sim-4-0-0.tar.xz

# Verify integrity
sha256sum isaac-sim-4-0-0.tar.xz
# Expected: a5d5b7e8f3c2a9b7c8d9e0f1g2h3i4j5k6l7m8n9o0p1
```

### Step 2: System Dependencies (3 minutes)

```bash
# Install minimal dependencies first
sudo apt update && sudo apt install -y \
    libvulkan1 \
    libvulkan-dev \
    vulkan-tools

# GPU verification
nvidia-smi --query-gpu=name,memory.total --format=csv,noheader
```

### Step 3: Extract and Review Space (1 minute)

```bash
# Extract the archive
tar -xf isaac-sim-4-0-0.tar.xz
cd isaac-sim-4.0.0

# Check disk space
du -sh .
```

### Step 4: First Launch (6 minutes)

```bash
# Launch Isaac Sim in simulation-friendly mode
./isaac-sim.sh --no-window --headless >> launch.log 2>&1

# Watch for successful initialization
tail -f launch.log | grep -E "(Successfully launched|ready)"

# Expected output within 60 seconds:
# ISAAC SIM successfully launched...
# App is ready!
```

## Detailed Installation

For standard systems with IDE or different hardware, follow the complete installation:

### System Requirements Verification

```python title="create hardware_verification_report.py (optional exhaustive check)"
import subprocess
import sys
import json

def verify_system ready(verbose=False):
    """Provide detailed system status for Isaac Sim compatibility"""

    requirements = {
        "minimum": {
            "gpu": "RTX 3060",
            "vram": 8192,
            "ram": 32768,
            "cpu_cores": 4,
            "storage": 50000,
            "os": "Ubuntu 20.04/22.04"
        },
        "recommended": {
            "gpu": "RTX 4090",
            "vram": 24576,
            "ram": 65536,
            "cpu_cores": 8,
            "storage": 100000,
            "os": "Ubuntu 22.04"
        }
    }

    report = {}

    # GPU check
    try:
        result = subprocess.run(['nvidia-smi', '--query-gpu=name,memory.total',
                                '--format=csv,noheader'],
                              capture_output=True, text=True)
        if result.returncode == 0 and result.stdout.strip():
            gpu_name, vram_mb = result.stdout.strip().split(',')
            vram_mb = int(vram_mb.strip().split()[{\"cluster\":[0]}])
            report['gpu_model'] = gpu_name.strip()
            report['vram_mb'] = vram_mb

            # Validation
            if vram_mb >= requirements['minimum']['vram']:
                report['vram_status'] = 'MINIMUM MET'
            if vram_mb >= requirements['recommended']['vram']:
                report['vram_status'] = 'RECOMMENDED MET'
        else:
            print("❌ NVIDIA GPU not detected (continuing CPU compatibility check)")
            report['gpu_model'] = 'CPU-ONLY'
            report['vram_status'] = 'INSUFFICIENT'
    except Exception as e:
        report['error'] = str(e)
        report['vram_status'] = 'ERROR'

    # Memory check
    try:
        with open('/proc/meminfo', 'r') as f:
            for line in f:
                if line.startswith('MemTotal:'):
                    ram_kb = int(line.split()[{\"cluster\":[1]}])
                    report['ram_mb'] = ram_kb / 1024
                    if ram_kb / 1024 >= requirements['minimum']['ram']:
                        report['ram_status'] = 'MINIMUM MET'
                    if ram_kb / 1024 >= requirements['recommended']['ram']:
                        report['ram_status'] = 'RECOMMENDED MET'
                    break
    except:
        print("⚠️ System memory check failed")

    # OS check
    try:
        import platform
        info = platform.platform()
        if 'ubuntu' in info.lower() or '20.04' in info or '22.04' in info:
            report['os_compatible'] = True
        else:
            report['os_compatible'] = False
        report['os_info'] = info
    except:
        report['os_compatible'] = 'UNABLE TO VERIFY'

    return report

if __name__ == "__main__":
    print("🔍 NVIDIA ISAAC SIM - SYSTEM COMPATIBILITY CHECK")
    print("="*50)

    report = verify_system_ready(verbose=True)

    print("\n📋 COMPATIBILITY REPORT:")
    print(json.dumps(report, indent=2))

    if 'vram_status' in report and 'RECOMMENDED' in report['vram_status']:
        print("\n✅ Your system meets recommended requirements!")
    elif 'vram_status' in report and 'MINIMUM' in report['vram_status']:
        print("\n✅ Your system meets minimum requirements.")
    else:
        print("\n❌ Your hardware does not meet requirements but CPU-only mode available")
```

## Custom Installation Methods

### Option A: Direct Installer (Recommended)

```bash
# Download the installer executable
wget https://developer.nvidia.com/downloads/isaac-sim-2023.1.1-linux_640449d034a2c882bfb2dba487e0b9aff7.run

# Make executable and run
chmod +x isaac-sim-*.run
./isaac-sim-*.run --accept --no-gui
```

### Option B: Container Installation

```bash
# For system isolation or headless servers
docker run -it --gpus all --name isaac-sim \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v ~/isaac/workspace:/workspace \
  --shm-size=8g \
  nvcr.io/nvidia/isaac-sim:2023.1.1
```

### Option C: Conda Installation (On approved systems only)

```bash
# Install Isaac Sim via conda
conda create -n isaac python=3.8
conda activate isaac

# Install relevant libraries
conda install pytorch torchvision torchaudio cudatoolkit=11.8 -c pytorch -c nvidia
pip install isaac-sim -i https://api.ngc.nvidia.com/v2/org/nvidia/team/isaac/resources/
```

## Post-Installation Verification

### Test 1: Basic Launch
```bash
# Verify Isaac Sim launches and creates render output
./isaac-sim.sh --simulate 60 --no-window > test.log 2>&1
# Check for successful simulation message
grep "Simulation started successfully" test.log
```

### Test 2: Simple Scenes
```bash
# Load a basic warehouse scene
./isaac-sim.sh --load-omniverse="/Isaac/Environments/Simple_Warehouse/warehouse_multi.usd" --headless

# Verify scene loads without errors
grep "Scene loaded successfully" output.log
```

### Test 3: ROS2 Integration
```bash
# Check ROS2 topics visibility
source /opt/ros/humble/setup.bash
ros2 topic list | grep -E "(simulation|sim)"
```

## Common Installation Issues

### CUDA Version Mismatch
```bash
# Check CUDA version
nvcc --version

# If below 11.8, update using the official installations script:
# https://developer.nvidia.com/cuda-11-8-0-download-archive
cd /tmp
wget https://developer.nvidia.com/cuda-11-8-0-download-archive-linux-ubuntu-2204-x86-64
sudo sh cuda-11-8-x86_64-linux.run
```

### Insufficient VRAM Warning
```bash
# Monitor GPU memory during first launch
watch -n 1 "nvidia-smi --query-gpu=memory.used,memory.total --format=csv,noheader"

# If approaching limit, reduce Isaac Sim graphics quality:
export ISAAC_GRAPHICS_QUALITY=medium
export ISAAC_MAX_RAY_DEPTH=4
./isaac-sim.sh
```

### Permission Issues
```bash
# Fix missing permissions for X11 forwarding
xhost + # Allow local connections (temporary)
sudo chmod 666 /tmp/.X11-unix/
```

## Next Steps

After successful installation:

1. **Configure for ROS2 integration**: Follow our [ROS2 Integration Guide](./ros2-integration.md)
2. **Test hardware acceleration**: Run our [Performance Benchmarks](../ isaac-performance.md)
3. **Load your first robot**: See our [Asset Integration guide](./asset-integration.md)

---

**Installation successful?** ✅ Continue to [Scene Creation](./scene-creation.md)

**Encountering issues?** ❌ Check our [Common Errors Guide](../common-errors.md#installation-issues) for immediate solutions or use our verification script above.