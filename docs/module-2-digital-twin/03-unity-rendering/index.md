---
id: unity-rendering
title: '03. Visual Simulation with Unity'
description: Build photorealistic environments for computer vision and human-robot interaction
sidebar_label: '03. Unity Visuals'
readingTime: 25
---

# 03. Visual Simulation with Unity

Unity creates stunning visual environments that look like real camera footage. This is perfect for training AI systems and human collaborations.

## Why Visual Simulation Matters

Real computer vision training data is expensive:
- Cameras cost hundreds or thousands of dollars
- Manually labeling thousands of images takes weeks
- Lighting conditions are hard to control

Unity solves this: **Generate unlimited perfect training data instantly.**

## Setting Up Your First Unity Project

### 1. Create a New Unity Project
- Install Unity Hub if you haven't already
- Create a new 3D project with Universal Render Pipeline (URP)
- Name it "RobotVisionTraining"

### 2. Import the Robotics Package

Unity has official robotics support:

```csharp
// Package Manager -> Add package from Git URL
com.unity.robotics.ros-tcp-connector
```

### 3. The Simplest Scene Setup

Create a basic warehouse scene:

1. **Floor**: GameObject -> 3D Object -> Plane (scale 10x10)
2. **Robot**: Right-click in Hierarchy -> Import Camera
3. **Lighting**: Window -> Rendering -> Lighting Settings
4. **Camera**: Position 5 meters above ground, look down

### 4. Add Some Props

Instantiate a few boxes from Unity Asset Store:
- Get free Robot Factory assets
- Add 5-10 cardboard boxes
- Position randomly around the floor

## Training Data Generation

With Unity ROS, stream camera data directly to ROS2:

```csharp
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;

public class CameraPublisher : MonoBehaviour
{
    [SerializeField] Camera targetCamera;
    [SerializeField] string topicName = "/camera/image";
    [SerializeField] string frameId = "camera_link";

    void Start()
    {
        ROSConnection.GetOrCreateInstance().RegisterPublisher<ImageMsg>(topicName);
        InvokeRepeating("TakeSnapshot", 0f, 0.033f); // 30 FPS
    }

    void TakeSnapshot()
    {
        // Capture camera image
        RenderTexture currentRT = RenderTexture.active;
        RenderTexture.active = targetCamera.targetTexture;
        targetCamera.Render();

        Texture2D image = new Texture2D(targetCamera.pixelWidth, targetCamera.pixelHeight, TextureFormat.RGB24, false);
        image.ReadPixels(new Rect(0, 0, targetCamera.pixelWidth, targetCamera.pixelHeight), 0, 0);
        image.Apply();

        RenderTexture.active = currentRT;

        // Convert to ROS image message
        ImageMsg rosImage = new ImageMsg();
        rosImage.height = (uint)image.height;
        rosImage.width = (uint)image.width;
        rosImage.encoding = "rgb8";  // RGB format
        rosImage.step = (uint)(image.width * 3);  // 3 bytes per pixel
        rosImage.data = image.GetRawTextureData();

        ROSConnection.GetOrCreateInstance().Send(topicName, rosImage);
    }
}```

## Domain Randomization

To bridge the sim-to-real gap, randomly vary your training data:

```csharp
// Vary lighting intensity
light.intensity = Random.Range(0.3f, 1.2f);

// Move camera position slightly
targetCamera.transform.Rotate(
    Random.Range(-10f, 10f),
    Random.Range(-5f, 5f),
    0f
);

// Change prop positions
foreach (GameObject box in warehouseBoxes)
{
    Vector3 randomPos = new Vector3(
        Random.Range(-10f, 10f),
        0.5f,
        Random.Range(-10f, 10f)
    );
    box.transform.position = randomPos;
}```

## Human-Robot Interaction

Unity excels at human simulation. Create a human avatar:

```csharp
public class HumanController : MonoBehaviour
{
    [Header("Gesture Recognition")]
    [SerializeField] string gestureTopic = "/gesture_commands";

    public enum Gestures
    {
        Stop = 0,
        Go = 1,
        TurnLeft = 2,
        TurnRight = 3
    }

    public void PlayGesture(Gestures gesture)
    {
        // Trigger avatar animation
        animator.SetInteger("GestureState", (int)gesture);

        // Send gesture to robot
        UInt32Msg gestureMsg = new UInt32Msg{
            data = (uint)gesture
        };
        ROSConnection.GetOrCreateInstance().Send(gestureTopic, gestureMsg);
    }
}```

## Testing Your Visual Environment

1. **Start Unity**: Open your RobotVisionTraining project
2. **Add Cameron**: Attach the CameraPublisher script to your camera
3. **Build a scene**: With robot, camera, and boxes
4. **Test data flow**: Ensure images publish to ROS2 topic
5. **Train a model**: Use the generated data with your algorithm

## Quick Verification

In Unity Console, you should see:
```
[INFO] Connected to ROS at localhost:10000
[INFO] Publishing /camera/image at 30fps
[INFO] Snapshot 1/100 taken: 1920x1080 RGB
[INFO] Ready for training...
```

And in ROS terminal:
```bash
rostopic echo /camera/image
```

## Key Takeaways

- Unity generates infinite, labeled training data
- Randomization prevents overfitting to simulation
- ROS2 integration brings simulation and reality together
- Use Unity when visual realism is important for your application

**Next Up**: Learn how to integrate both simulation approaches into a complete workflow.

---

*Stuck? Unity's Package Manager makes adding robot-specific packages easy. Search "robotics" to find the latest tools.*"}ල line count exceed, please continue...