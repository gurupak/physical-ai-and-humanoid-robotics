---
id: 02-installation
title: "Installation & Setup"
sidebar_label: "Installation"
sidebar_position: 2
sidebar_custom_props:
  difficulty: "Beginner"
  readingTime: "10 minutes"
  handsOnTime: "30 minutes"
---

import Quiz from '@site/src/components/Quiz';
import Callout from '@site/src/components/Callout';

# Installation & Setup

Before you can build ROS 2 applications, you need to install the framework on your system. This sub-chapter provides streamlined installation instructions and verification steps to get you up and running quickly.

---

## Recommended Setup

For this book, we recommend:

- **Operating System**: Ubuntu 22.04 LTS (Jammy Jellyfish)
- **ROS 2 Distribution**: **Humble Hawksbill** (LTS release, supported until May 2027)
- **Python Version**: 3.10+ (comes with Ubuntu 22.04)

<Callout type="info" title="Why Ubuntu 22.04 and Humble?">
- **Long-Term Support (LTS)**: Both Ubuntu 22.04 and ROS 2 Humble are LTS releases with 5 years of support
- **Stability**: Most ROS 2 packages are tested on Ubuntu LTS versions
- **Community**: Largest user base means better documentation and community support

If you're using Windows or macOS, see the [Alternative Platforms](#alternative-platforms) section below.
</Callout>

---

## Installation Steps (Ubuntu 22.04)

### Step 1: Set Locale

Ensure your system supports UTF-8:

```bash title="Terminal"
locale  # check current locale

# If not UTF-8, set it:
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# Verify
locale
```

### Step 2: Add ROS 2 APT Repository

```bash
# Enable Ubuntu Universe repository
sudo apt install software-properties-common
sudo add-apt-repository universe

# Add ROS 2 GPG key
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

# Add repository to sources list
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

### Step 3: Install ROS 2 Humble

```bash
# Update package index
sudo apt update

# Upgrade existing packages
sudo apt upgrade

# Install ROS 2 Humble Desktop (includes rviz, demos, tutorials)
sudo apt install ros-humble-desktop
```

**Installation Time**: ~5-10 minutes depending on internet speed.

<Callout type="tip" title="Minimal vs. Desktop Install">
- **Desktop** (`ros-humble-desktop`): Includes visualization tools (rviz2, rqt), demos, and tutorials. **Recommended for learning.**
- **Base** (`ros-humble-ros-base`): Minimal installation without GUI tools. Use for production systems or headless servers.
- **Full** (`ros-humble-desktop-full`): Desktop + simulation (Gazebo) and perception libraries. Largest download (~2GB+).
</Callout>

### Step 4: Install Development Tools

```bash
# Install colcon build tool
sudo apt install python3-colcon-common-extensions

# Install vcstool for managing repositories
sudo apt install python3-vcstool

# Install rosdep for dependency management
sudo apt install python3-rosdep
sudo rosdep init
rosdep update
```

---

## Environment Setup

### Sourcing ROS 2

Every time you open a new terminal, you need to "source" the ROS 2 setup script to access ROS 2 commands:

```bash
source /opt/ros/humble/setup.bash
```

**To avoid doing this manually every time**, add it to your `.bashrc`:

```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

<Quiz
  question="Why do you need to source the ROS 2 setup script?"
  options={[
    "To install additional ROS 2 packages",
    "To add ROS 2 commands and environment variables to your current shell session",
    "To start the ROS 2 master node",
    "To compile C++ code into executables"
  ]}
  correctAnswer={1}
  explanation="Sourcing the setup script adds ROS 2 executables to your PATH and sets environment variables like ROS_DISTRO. Without sourcing, commands like 'ros2' won't be found. ROS 2 doesn't have a master node (unlike ROS 1), and sourcing doesn't install packages or compile code."
  difficulty="medium"
/>

---

## Verification

### Check ROS 2 Version

```bash
ros2 --version
```

**Expected Output**:
```
ros2 cli version: 0.20.3
```

(Version number may vary slightly depending on updates)

### Test with a Demo

Run a simple "talker" demo:

```bash
# Terminal 1: Start talker node
ros2 run demo_nodes_cpp talker
```

**Expected Output**:
```
[INFO] [1638360000.123456789] [talker]: Publishing: 'Hello World: 1'
[INFO] [1638360000.623456789] [talker]: Publishing: 'Hello World: 2'
[INFO] [1638360001.123456789] [talker]: Publishing: 'Hello World: 3'
...
```

Open a **second terminal** and run the listener:

```bash
# Terminal 2: Start listener node
ros2 run demo_nodes_py listener
```

**Expected Output**:
```
[INFO] [1638360000.234567890] [listener]: I heard: [Hello World: 1]
[INFO] [1638360000.734567890] [listener]: I heard: [Hello World: 2]
[INFO] [1638360001.234567890] [listener]: I heard: [Hello World: 3]
...
```

If you see messages being exchanged, **congratulations!** 🎉 ROS 2 is installed correctly.

<Callout type="warning" title="Sourcing in New Terminals">
If the listener doesn't receive messages, make sure you sourced ROS 2 in **both terminals**:

```bash
source /opt/ros/humble/setup.bash
```

Or add it to `.bashrc` so it happens automatically for all new terminals.
</Callout>

---

## Troubleshooting

### Error: `ros2: command not found`

**Cause**: ROS 2 setup script not sourced.

**Solution**:
```bash
source /opt/ros/humble/setup.bash
```

Or add to `.bashrc` for persistence.

---

### Error: `GPG error: ... NO_PUBKEY`

**Cause**: ROS 2 repository key not added correctly.

**Solution**: Re-run Step 2 (Add ROS 2 APT Repository) and ensure the GPG key is added:
```bash
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
```

---

### Error: `Package 'ros-humble-desktop' has no installation candidate`

**Cause**: APT sources not updated or wrong Ubuntu version.

**Solution**:
1. Verify Ubuntu version: `lsb_release -a` (should show 22.04)
2. Update package lists: `sudo apt update`
3. Check `/etc/apt/sources.list.d/ros2.list` exists

---

<Quiz
  question="What command do you use to verify ROS 2 is installed correctly?"
  options={[
    "roscore",
    "ros2 --version",
    "python3 -m ros2",
    "rosdep install ros2"
  ]}
  correctAnswer={1}
  explanation="The command 'ros2 --version' prints the installed ROS 2 CLI version. 'roscore' is a ROS 1 command (ROS 2 has no central master). The other options are not valid verification commands."
  difficulty="easy"
/>

---

## Alternative Platforms

### Windows 10/11

ROS 2 supports Windows natively!

1. **Install Visual Studio 2019** (Community Edition, with C++ tools)
2. **Download ROS 2 Humble Windows installer**: [ROS 2 Windows Downloads](https://docs.ros.org/en/humble/Installation/Windows-Install-Binary.html)
3. **Run installer** and follow prompts
4. **Source setup**: `call C:\ros2_humble\local_setup.bat` in Command Prompt

<Callout type="info" title="Windows Limitations">
Some ROS 2 packages may not be available on Windows due to Linux-specific dependencies. For full compatibility, consider using **WSL 2** (Windows Subsystem for Linux) with Ubuntu 22.04.
</Callout>

### macOS

ROS 2 Humble supports macOS via **Homebrew**:

```bash
# Install Homebrew if not already installed
/bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)"

# Install ROS 2
brew install ros/humble/ros-humble-desktop
```

**Note**: macOS support is community-driven and may lag behind Linux. For production systems, Ubuntu is recommended.

### Docker (Any Platform)

Use an official ROS 2 Docker image for platform-agnostic setup:

```bash
# Pull ROS 2 Humble image
docker pull osrf/ros:humble-desktop

# Run interactive container
docker run -it osrf/ros:humble-desktop
```

Inside the container, ROS 2 is pre-installed and sourced. This is ideal for:
- **Development on non-Linux systems**
- **CI/CD pipelines**
- **Isolated testing environments**

---

## What You've Learned

In this sub-chapter, you've:

- ✅ Installed **ROS 2 Humble** on Ubuntu 22.04 (or alternative platform)
- ✅ Configured your **environment** to source ROS 2 automatically
- ✅ Verified the installation with **demo talker/listener** nodes
- ✅ Learned **troubleshooting** steps for common installation issues

---

## Next Steps

Now that ROS 2 is installed, you're ready to write your first node!

**Continue to**: [Your First Node](./03-first-node.md)

---

## Additional Resources

:::info Official Installation Guides
- [Ubuntu Installation (Detailed)](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)
- [Windows Installation](https://docs.ros.org/en/humble/Installation/Windows-Install-Binary.html)
- [macOS Installation](https://docs.ros.org/en/humble/Installation/macOS-Install-Binary.html)
- [Docker Images](https://hub.docker.com/r/osrf/ros)
:::
