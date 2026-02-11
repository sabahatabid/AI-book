---
sidebar_position: 3
title: Unity for Robotics
---

# Unity for High-Fidelity Rendering

## Introduction

While Gazebo excels at physics simulation, Unity provides photorealistic rendering and advanced visualization capabilities. Unity's Robotics Hub bridges the gap between Unity and ROS 2.

## Why Unity for Robotics?

### Advantages

1. **Photorealistic Graphics** - High-quality rendering
2. **VR/AR Support** - Immersive experiences
3. **Asset Store** - Thousands of ready-made assets
4. **Cross-Platform** - Windows, Mac, Linux, Mobile
5. **Game Engine Features** - Advanced lighting, physics, AI

### Use Cases

- Human-robot interaction visualization
- Training data generation
- Marketing and demonstrations
- VR teleoperation
- Simulation visualization

## Unity Robotics Hub

### Installation

1. **Install Unity Hub**
   - Download from unity.com
   - Install Unity 2021.3 LTS or newer

2. **Create New Project**
   - 3D Template
   - Name: "RoboticsSimulation"

3. **Install Robotics Packages**

```
Window → Package Manager → Add package from git URL:
- com.unity.robotics.urdf-importer
- com.unity.robotics.ros-tcp-connector
```

## Importing URDF into Unity

### URDF Importer

```csharp
// In Unity Editor:
// 1. Assets → Import Robot from URDF
// 2. Select your .urdf file
// 3. Configure import settings
// 4. Click Import
```

### Import Settings

- **Axis Type**: Y-Up (Unity) or Z-Up (ROS)
- **Mesh Decomposer**: VHACD for collision
- **Convex Method**: Inherit or Force Convex
- **Mesh Scale Factor**: Usually 1.0

## ROS-Unity Communication

### TCP Endpoint Setup

```csharp
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Std;

public class ROSConnection : MonoBehaviour
{
    private ROSConnection ros;
    
    void Start()
    {
        // Connect to ROS
        ros = ROSConnection.GetOrCreateInstance();
        ros.Connect("192.168.1.100", 10000);
        
        // Subscribe to topic
        ros.Subscribe<StringMsg>("chatter", MessageCallback);
        
        // Register publisher
        ros.RegisterPublisher<StringMsg>("unity_output");
    }
    
    void MessageCallback(StringMsg message)
    {
        Debug.Log($"Received: {message.data}");
    }
    
    void PublishMessage()
    {
        StringMsg msg = new StringMsg { data = "Hello from Unity!" };
        ros.Publish("unity_output", msg);
    }
}
```

### ROS 2 Bridge Setup

On ROS 2 side:

```bash
# Install ROS TCP Endpoint
sudo apt install ros-humble-ros-tcp-endpoint

# Run the endpoint
ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p ROS_IP:=0.0.0.0
```

## Creating Realistic Environments

### Lighting Setup

```csharp
using UnityEngine;

public class LightingManager : MonoBehaviour
{
    public Light directionalLight;
    
    void Start()
    {
        // Configure sun
        directionalLight.type = LightType.Directional;
        directionalLight.intensity = 1.0f;
        directionalLight.shadows = LightShadows.Soft;
        
        // Set ambient lighting
        RenderSettings.ambientMode = UnityEngine.Rendering.AmbientMode.Skybox;
        RenderSettings.ambientIntensity = 0.5f;
    }
}
```

### Post-Processing

```csharp
using UnityEngine.Rendering.PostProcessing;

public class PostProcessingSetup : MonoBehaviour
{
    void Start()
    {
        var volume = gameObject.AddComponent<PostProcessVolume>();
        volume.isGlobal = true;
        
        var profile = ScriptableObject.CreateInstance<PostProcessProfile>();
        
        // Add effects
        var bloom = profile.AddSettings<Bloom>();
        bloom.enabled.Override(true);
        bloom.intensity.Override(0.5f);
        
        var ao = profile.AddSettings<AmbientOcclusion>();
        ao.enabled.Override(true);
        
        volume.profile = profile;
    }
}
```

## Robot Control in Unity

### Joint Controller

```csharp
using UnityEngine;

public class JointController : MonoBehaviour
{
    public ArticulationBody joint;
    public float targetPosition;
    public float speed = 1.0f;
    
    void FixedUpdate()
    {
        if (joint != null)
        {
            var drive = joint.xDrive;
            drive.target = targetPosition;
            drive.stiffness = 10000;
            drive.damping = 100;
            drive.forceLimit = float.MaxValue;
            joint.xDrive = drive;
        }
    }
    
    public void SetTarget(float angle)
    {
        targetPosition = angle * Mathf.Rad2Deg;
    }
}
```

### Robot Movement

```csharp
using UnityEngine;
using RosMessageTypes.Geometry;

public class RobotMover : MonoBehaviour
{
    private ROSConnection ros;
    public float speed = 1.0f;
    
    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.Subscribe<TwistMsg>("cmd_vel", OnVelocityCommand);
    }
    
    void OnVelocityCommand(TwistMsg twist)
    {
        // Apply velocity to robot
        Vector3 linear = new Vector3(
            (float)twist.linear.x,
            0,
            (float)twist.linear.z
        );
        
        float angular = (float)twist.angular.y;
        
        transform.Translate(linear * Time.deltaTime * speed);
        transform.Rotate(0, angular * Time.deltaTime * Mathf.Rad2Deg, 0);
    }
}
```

## Camera Systems

### Follow Camera

```csharp
using UnityEngine;

public class FollowCamera : MonoBehaviour
{
    public Transform target;
    public Vector3 offset = new Vector3(0, 2, -5);
    public float smoothSpeed = 0.125f;
    
    void LateUpdate()
    {
        if (target != null)
        {
            Vector3 desiredPosition = target.position + offset;
            Vector3 smoothedPosition = Vector3.Lerp(
                transform.position,
                desiredPosition,
                smoothSpeed
            );
            transform.position = smoothedPosition;
            transform.LookAt(target);
        }
    }
}
```

### First-Person Camera

```csharp
using UnityEngine;
using RosMessageTypes.Sensor;

public class RobotCamera : MonoBehaviour
{
    private ROSConnection ros;
    private Camera cam;
    
    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        cam = GetComponent<Camera>();
        
        // Publish camera images
        InvokeRepeating("PublishImage", 0, 0.1f); // 10 Hz
    }
    
    void PublishImage()
    {
        RenderTexture rt = new RenderTexture(640, 480, 24);
        cam.targetTexture = rt;
        cam.Render();
        
        Texture2D image = new Texture2D(640, 480, TextureFormat.RGB24, false);
        RenderTexture.active = rt;
        image.ReadPixels(new Rect(0, 0, 640, 480), 0, 0);
        image.Apply();
        
        // Convert to ROS message
        ImageMsg msg = new ImageMsg
        {
            height = 480,
            width = 640,
            encoding = "rgb8",
            data = image.GetRawTextureData()
        };
        
        ros.Publish("camera/image_raw", msg);
        
        cam.targetTexture = null;
        RenderTexture.active = null;
        Destroy(rt);
    }
}
```

## Sensor Simulation

### LIDAR Simulation

```csharp
using UnityEngine;
using RosMessageTypes.Sensor;

public class LidarSensor : MonoBehaviour
{
    public int numRays = 360;
    public float maxRange = 10.0f;
    public float scanRate = 10.0f; // Hz
    
    private ROSConnection ros;
    
    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        InvokeRepeating("PublishScan", 0, 1.0f / scanRate);
    }
    
    void PublishScan()
    {
        float[] ranges = new float[numRays];
        float angleIncrement = 2 * Mathf.PI / numRays;
        
        for (int i = 0; i < numRays; i++)
        {
            float angle = i * angleIncrement;
            Vector3 direction = new Vector3(
                Mathf.Cos(angle),
                0,
                Mathf.Sin(angle)
            );
            
            RaycastHit hit;
            if (Physics.Raycast(transform.position, direction, out hit, maxRange))
            {
                ranges[i] = hit.distance;
            }
            else
            {
                ranges[i] = float.PositiveInfinity;
            }
        }
        
        LaserScanMsg msg = new LaserScanMsg
        {
            angle_min = 0,
            angle_max = 2 * Mathf.PI,
            angle_increment = angleIncrement,
            range_min = 0.1f,
            range_max = maxRange,
            ranges = ranges
        };
        
        ros.Publish("scan", msg);
    }
}
```

## VR Integration

### VR Robot Control

```csharp
using UnityEngine;
using UnityEngine.XR;

public class VRRobotController : MonoBehaviour
{
    public Transform leftHand;
    public Transform rightHand;
    private ROSConnection ros;
    
    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
    }
    
    void Update()
    {
        // Get VR controller input
        InputDevice leftDevice = InputDevices.GetDeviceAtXRNode(XRNode.LeftHand);
        InputDevice rightDevice = InputDevices.GetDeviceAtXRNode(XRNode.RightHand);
        
        // Read trigger values
        float leftTrigger, rightTrigger;
        leftDevice.TryGetFeatureValue(CommonUsages.trigger, out leftTrigger);
        rightDevice.TryGetFeatureValue(CommonUsages.trigger, out rightTrigger);
        
        // Control robot based on VR input
        if (leftTrigger > 0.5f)
        {
            // Trigger robot action
            PublishCommand("grasp");
        }
    }
    
    void PublishCommand(string command)
    {
        // Publish to ROS
        var msg = new RosMessageTypes.Std.StringMsg { data = command };
        ros.Publish("robot_command", msg);
    }
}
```

## Performance Optimization

### Tips

1. **Use LOD (Level of Detail)** - Reduce polygon count at distance
2. **Occlusion Culling** - Don't render hidden objects
3. **Bake Lighting** - Pre-compute static lighting
4. **Object Pooling** - Reuse objects instead of instantiating
5. **Reduce Draw Calls** - Batch similar objects

```csharp
// LOD Setup
LODGroup lodGroup = gameObject.AddComponent<LODGroup>();
LOD[] lods = new LOD[3];

lods[0] = new LOD(0.6f, highDetailRenderers);
lods[1] = new LOD(0.3f, mediumDetailRenderers);
lods[2] = new LOD(0.1f, lowDetailRenderers);

lodGroup.SetLODs(lods);
lodGroup.RecalculateBounds();
```

## Next Steps

Learn about sensor simulation in detail in [Sensor Simulation](sensor-simulation.md).
