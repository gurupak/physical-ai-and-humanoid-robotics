# Quick Start: VLA Voice Command in 15 Minutes

**Feature**: 008-vla-module | **Date**: 2025-12-09  
**Purpose**: Hands-on demonstration to build a voice-controlled robot in 15 minutes

---

## Goal

Build a robot that listens to your voice command, transcribes it using OpenAI Whisper, and publishes the command to a ROS 2 topic. This demonstrates the first step in the VLA pipeline: Voice → Text.

**Expected Outcome**: Speak "Move forward" → Robot receives transcribed command within 2 seconds

---

## Prerequisites

- ROS 2 Humble installed and sourced (`source /opt/ros/humble/setup.bash`)
- Python 3.11+ with pip
- Microphone connected to your computer
- ~10 GB disk space (for Whisper model download)

---

## Step-by-Step Guide

### Step 1: Install Dependencies (2 minutes)

```bash
# Install OpenAI Whisper
pip install openai-whisper

# Install audio libraries
pip install pyaudio sounddevice

# Verify installation
python -c "import whisper; print(whisper.__version__)"
```

**Expected output**: Version number (e.g., `20231117`)

**Troubleshooting**:
- If `pyaudio` fails: Install PortAudio first
  - Ubuntu: `sudo apt-get install portaudio19-dev`
  - macOS: `brew install portaudio`

---

### Step 2: Test Your Microphone (1 minute)

```bash
# List available audio devices
python -c "import sounddevice; print(sounddevice.query_devices())"
```

**Expected output**: List of input/output devices with names and indices

**Note the device index** for your microphone (usually 0 or 1)

---

### Step 3: Test Whisper Transcription (3 minutes)

Create a test script to verify Whisper works:

```python
# test_whisper.py
import whisper
import sounddevice as sd
import numpy as np

# Load Whisper model (downloads ~500MB on first run)
print("Loading Whisper model...")
model = whisper.load_model("small")  # ~500MB, good accuracy/speed balance

# Record 5 seconds of audio
print("Recording for 5 seconds... Speak a command!")
sample_rate = 16000
duration = 5
audio = sd.rec(int(duration * sample_rate), samplerate=sample_rate, channels=1, dtype=np.float32)
sd.wait()  # Wait for recording to finish

# Transcribe
print("Transcribing...")
audio_flat = audio.flatten()
result = model.transcribe(audio_flat, language='en')

print(f"You said: {result['text']}")
```

Run it:
```bash
python test_whisper.py
```

**Expected behavior**:
1. "Loading Whisper model..." (10-30 seconds first time)
2. "Recording for 5 seconds... Speak a command!"
3. **You speak**: "Move forward"
4. "Transcribing..." (1-2 seconds)
5. "You said: Move forward"

**Troubleshooting**:
- **"No audio detected"**: Check microphone permissions, verify device index
- **"Transcription is blank"**: Speak louder, check microphone volume settings
- **"Wrong transcription"**: Use quieter environment, speak clearly

---

### Step 4: Create ROS 2 Voice Node (5 minutes)

Create a ROS 2 package:

```bash
# Create workspace and package
mkdir -p ~/vla_ws/src
cd ~/vla_ws/src
ros2 pkg create --build-type ament_python vla_voice --dependencies rclpy std_msgs

# Navigate to package
cd vla_voice/vla_voice
```

Create the voice node (`voice_command_node.py`):

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import whisper
import sounddevice as sd
import numpy as np
import threading

class VoiceCommandNode(Node):
    def __init__(self):
        super().__init__('voice_command_node')
        
        # Publisher for transcribed commands
        self.text_pub = self.create_publisher(String, '/voice/command', 10)
        
        # Load Whisper model
        self.get_logger().info("Loading Whisper model...")
        self.model = whisper.load_model("small")
        self.get_logger().info("Whisper ready!")
        
        # Audio parameters
        self.sample_rate = 16000
        self.duration = 5  # seconds to record
        
        # Start listening thread
        self.listening = True
        self.listen_thread = threading.Thread(target=self.listen_loop)
        self.listen_thread.start()
        
        self.get_logger().info("Voice command node ready. Speak commands!")
    
    def listen_loop(self):
        """Continuously listen for voice commands"""
        while self.listening and rclpy.ok():
            try:
                # Record audio
                self.get_logger().info("Listening... (speak for 5 seconds)")
                audio = sd.rec(
                    int(self.duration * self.sample_rate),
                    samplerate=self.sample_rate,
                    channels=1,
                    dtype=np.float32
                )
                sd.wait()
                
                # Transcribe
                self.get_logger().info("Transcribing...")
                audio_flat = audio.flatten()
                result = self.model.transcribe(audio_flat, language='en')
                
                # Publish command
                command_text = result['text'].strip()
                if command_text:  # Only publish non-empty
                    msg = String()
                    msg.data = command_text
                    self.text_pub.publish(msg)
                    self.get_logger().info(f"Published command: '{command_text}'")
                
            except Exception as e:
                self.get_logger().error(f"Error in listen loop: {e}")
    
    def destroy_node(self):
        """Clean shutdown"""
        self.listening = False
        if self.listen_thread.is_alive():
            self.listen_thread.join(timeout=2.0)
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = VoiceCommandNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

Update `setup.py` to add the entry point:

```python
# In setup.py, add to entry_points:
entry_points={
    'console_scripts': [
        'voice_command_node = vla_voice.voice_command_node:main',
    ],
},
```

Build the package:

```bash
cd ~/vla_ws
colcon build --packages-select vla_voice
source install/setup.bash
```

---

### Step 5: Run the Voice Node (2 minutes)

Launch the node:

```bash
ros2 run vla_voice voice_command_node
```

**Expected output**:
```
[INFO] [voice_command_node]: Loading Whisper model...
[INFO] [voice_command_node]: Whisper ready!
[INFO] [voice_command_node]: Voice command node ready. Speak commands!
[INFO] [voice_command_node]: Listening... (speak for 5 seconds)
```

**Speak a command** (e.g., "Move forward") during the listening window.

**Expected output after speech**:
```
[INFO] [voice_command_node]: Transcribing...
[INFO] [voice_command_node]: Published command: 'Move forward'
[INFO] [voice_command_node]: Listening... (speak for 5 seconds)
```

---

### Step 6: Verify ROS 2 Topic (1 minute)

In a **new terminal**:

```bash
# Source ROS 2
source /opt/ros/humble/setup.bash
source ~/vla_ws/install/setup.bash

# Echo the topic to see published commands
ros2 topic echo /voice/command
```

**Expected output** (when you speak):
```
data: 'Move forward'
---
data: 'Turn left'
---
```

---

### Step 7: Visualize with rqt_graph (1 minute - Optional)

```bash
ros2 run rqt_graph rqt_graph
```

**Expected visual**:
- Node: `/voice_command_node`
- Topic: `/voice/command` (String)
- Publisher: `voice_command_node` → `/voice/command`

---

## Success Criteria ✅

You've successfully completed the quick start if:

1. ✅ Whisper model loads without errors
2. ✅ Voice node publishes transcribed text to `/voice/command` topic
3. ✅ Transcription accuracy >90% for clear speech
4. ✅ Latency from speech end to publication <3 seconds

---

## Next Steps

### Immediate Improvements
1. **Add voice activity detection (VAD)**: Only transcribe when speech detected (reduces CPU usage)
2. **Confidence filtering**: Reject low-confidence transcriptions
3. **Command vocabulary**: Limit to predefined robot commands

### Integration
- **Connect to robot control**: Subscribe to `/voice/command` in a robot controller node
- **Add LLM planning**: Send transcribed text to LLM for action planning (Chapter 4.3)
- **Visual feedback**: Display transcription on screen or robot display

### Advanced
- **Streaming transcription**: Use WhisperLive for real-time transcription
- **Custom wake word**: Add "Hey robot" activation
- **Noise cancellation**: Implement audio preprocessing

---

## Troubleshooting

### Issue: "Model download is slow"
**Solution**: 
- Use smaller model: `whisper.load_model("tiny")` (39MB, fast download)
- Download manually and point to local path:
  ```python
  model = whisper.load_model("small", download_root="/path/to/models")
  ```

### Issue: "Transcription takes >5 seconds"
**Solution**:
- Use GPU: Ensure PyTorch with CUDA installed
  ```bash
  pip install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu118
  ```
- Switch to smaller model (`tiny` or `base`)
- Reduce recording duration to 3 seconds

### Issue: "Background noise causes incorrect transcription"
**Solution**:
- Use noise-cancelling microphone
- Record in quieter environment
- Add preprocessing (lowpass filter, noise gate)
- Try medium model for better noise robustness

### Issue: "Node crashes with memory error"
**Solution**:
- Insufficient RAM (need ~2GB free)
- Use `tiny` model (requires less memory)
- Close other applications

---

## What You Learned

🎓 **Skills Acquired**:
- Installed and used OpenAI Whisper for speech recognition
- Created a ROS 2 node with Python
- Published messages to ROS 2 topics
- Integrated audio processing with robotics middleware

🤖 **VLA Pipeline Progress**:
- ✅ Voice → Text (Whisper) - **COMPLETED**
- ⏸️ Text → Plan (LLM) - Next in Chapter 4.3
- ⏸️ Vision → Features - Next in Chapter 4.4
- ⏸️ Plan → Actions - Next in Chapter 4.5

---

## Time Breakdown

| Step | Time | Activity |
|------|------|----------|
| 1 | 2 min | Install dependencies |
| 2 | 1 min | Test microphone |
| 3 | 3 min | Test Whisper standalone |
| 4 | 5 min | Create ROS 2 voice node |
| 5 | 2 min | Run and test node |
| 6 | 1 min | Verify topic |
| 7 | 1 min | Visualize (optional) |
| **Total** | **15 min** | Voice-controlled robot! |

---

## Code Repository

Full code available at:
```
~/vla_ws/src/vla_voice/
├── vla_voice/
│   ├── __init__.py
│   └── voice_command_node.py
├── setup.py
├── package.xml
└── README.md
```

---

Congratulations! You've built the foundation of a voice-controlled robot. Continue to **Chapter 4.3** to add cognitive planning with LLMs. 🚀
