---
sidebar_position: 4
title: Sensor Simulation
---

# Sensor Simulation

## Introduction

Accurate sensor simulation is crucial for developing robust robotic systems. This chapter covers simulating the key sensors used in humanoid robotics.

## Types of Sensors

### 1. LIDAR (Light Detection and Ranging)

**Purpose**: Distance measurement and mapping

**Characteristics**:
- 360° or limited field of view
- Range: 0.1m to 100m+
- Update rate: 5-40 Hz
- Point cloud output

### 2. Depth Cameras (RGB-D)

**Purpose**: 3D vision and object detection

**Types**:
- Structured light (Kinect)
- Time-of-flight (RealSense)
- Stereo cameras

**Output**: RGB image + depth map

### 3. IMU (Inertial Measurement Unit)

**Purpose**: Orientation and acceleration

**Components**:
- 3-axis accelerometer
- 3-axis gyroscope
- 3-axis magnetometer (optional)

**Output**: Linear acceleration, angular velocity

### 4. Force/Torque Sensors

**Purpose**: Contact detection and manipulation

**Location**: Joints, feet, hands

**Output**: 6-DOF force and torque vectors

## LIDAR Simulation in Gazebo

### 2D LIDAR

```xml
<gazebo reference="laser_link">
  <sensor type="ray" name="laser_scanner">
    <pose>0 0 0 0 0 0</pose>
    <visualize>true</visualize>
    <update_rate>40</update_rate>
    <ray>
      <scan>
        <horizontal>
          <samples>720</samples>
          <resolution>1</resolution>
          <min_angle>-3.14159</min_angle>
          <max_angle>3.14159</max_angle>
        </horizontal>
      </scan>
      <range>
        <min>0.10</min>
        <max>30.0</max>
        <resolution>0.01</resolution>
      </range>
      <noise>
        <type>gaussian</type>
        <mean>0.0</mean>
        <stddev>0.01</stddev>
      </noise>
    </ray>
    <plugin name="gazebo_ros_laser" filename="libgazebo_ros_ray_sensor.so">
      <ros>
        <namespace>/robot</namespace>
        <remapping>~/out:=scan</remapping>
      </ros>
      <output_type>sensor_msgs/LaserScan</output_type>
      <frame_name>laser_link</frame_name>
    </plugin>
  </sensor>
</gazebo>
```

### 3D LIDAR (Velodyne-style)

```xml
<gazebo reference="velodyne_link">
  <sensor type="ray" name="velodyne">
    <pose>0 0 0 0 0 0</pose>
    <visualize>false</visualize>
    <update_rate>10</update_rate>
    <ray>
      <scan>
        <horizontal>
          <samples>1800</samples>
          <resolution>1</resolution>
          <min_angle>-3.14159</min_angle>
          <max_angle>3.14159</max_angle>
        </horizontal>
        <vertical>
          <samples>16</samples>
          <resolution>1</resolution>
          <min_angle>-0.2617</min_angle>
          <max_angle>0.2617</max_angle>
        </vertical>
      </scan>
      <range>
        <min>0.9</min>
        <max>130.0</max>
        <resolution>0.001</resolution>
      </range>
    </ray>
    <plugin name="gazebo_ros_laser_controller" filename="libgazebo_ros_ray_sensor.so">
      <ros>
        <remapping>~/out:=points</remapping>
      </ros>
      <output_type>sensor_msgs/PointCloud2</output_type>
      <frame_name>velodyne_link</frame_name>
    </plugin>
  </sensor>
</gazebo>
```

## Depth Camera Simulation

### RGB-D Camera (Kinect/RealSense)

```xml
<gazebo reference="camera_link">
  <sensor type="depth" name="depth_camera">
    <update_rate>30</update_rate>
    <camera>
      <horizontal_fov>1.047198</horizontal_fov>
      <image>
        <width>640</width>
        <height>480</height>
        <format>R8G8B8</format>
      </image>
      <clip>
        <near>0.05</near>
        <far>8.0</far>
      </clip>
    </camera>
    <plugin name="depth_camera_controller" filename="libgazebo_ros_camera.so">
      <ros>
        <namespace>/robot</namespace>
        <remapping>~/image_raw:=rgb/image_raw</remapping>
        <remapping>~/depth/image_raw:=depth/image_raw</remapping>
        <remapping>~/points:=depth/points</remapping>
      </ros>
      <camera_name>depth_camera</camera_name>
      <frame_name>camera_link</frame_name>
      <hack_baseline>0.07</hack_baseline>
      <min_depth>0.05</min_depth>
      <max_depth>8.0</max_depth>
    </plugin>
  </sensor>
</gazebo>
```

### Processing Depth Data

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2
from cv_bridge import CvBridge
import numpy as np

class DepthProcessor(Node):
    def __init__(self):
        super().__init__('depth_processor')
        self.bridge = CvBridge()
        
        self.subscription = self.create_subscription(
            Image,
            '/robot/depth/image_raw',
            self.depth_callback,
            10
        )
        
    def depth_callback(self, msg):
        # Convert to numpy array
        depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        
        # Process depth data
        # Find closest point
        min_depth = np.min(depth_image[depth_image > 0])
        
        # Detect obstacles
        obstacle_mask = (depth_image < 1.0) & (depth_image > 0)
        
        if np.any(obstacle_mask):
            self.get_logger().warn(f'Obstacle detected at {min_depth:.2f}m')
```

## IMU Simulation

### IMU Sensor Configuration

```xml
<gazebo reference="imu_link">
  <sensor name="imu_sensor" type="imu">
    <always_on>true</always_on>
    <update_rate>100</update_rate>
    <imu>
      <angular_velocity>
        <x>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>2e-4</stddev>
          </noise>
        </x>
        <y>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>2e-4</stddev>
          </noise>
        </y>
        <z>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>2e-4</stddev>
          </noise>
        </z>
      </angular_velocity>
      <linear_acceleration>
        <x>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>1.7e-2</stddev>
          </noise>
        </x>
        <y>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>1.7e-2</stddev>
          </noise>
        </y>
        <z>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>1.7e-2</stddev>
          </noise>
        </z>
      </linear_acceleration>
    </imu>
    <plugin name="imu_plugin" filename="libgazebo_ros_imu_sensor.so">
      <ros>
        <namespace>/robot</namespace>
        <remapping>~/out:=imu</remapping>
      </ros>
      <initial_orientation_as_reference>false</initial_orientation_as_reference>
    </plugin>
  </sensor>
</gazebo>
```

### IMU Data Processing

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import numpy as np

class IMUProcessor(Node):
    def __init__(self):
        super().__init__('imu_processor')
        
        self.subscription = self.create_subscription(
            Imu,
            '/robot/imu',
            self.imu_callback,
            10
        )
        
        self.orientation_history = []
        
    def imu_callback(self, msg):
        # Extract orientation (quaternion)
        q = msg.orientation
        
        # Extract angular velocity
        w = msg.angular_velocity
        
        # Extract linear acceleration
        a = msg.linear_acceleration
        
        # Detect falls
        if abs(a.z) < 5.0:  # Less than half gravity
            self.get_logger().warn('Possible fall detected!')
        
        # Detect rapid rotation
        angular_magnitude = np.sqrt(w.x**2 + w.y**2 + w.z**2)
        if angular_magnitude > 2.0:  # rad/s
            self.get_logger().warn('Rapid rotation detected!')
```

## Force/Torque Sensors

### Contact Sensor

```xml
<gazebo reference="foot_link">
  <sensor name="foot_contact" type="contact">
    <contact>
      <collision>foot_collision</collision>
    </contact>
    <update_rate>100</update_rate>
    <plugin name="foot_contact_plugin" filename="libgazebo_ros_bumper_sensor.so">
      <ros>
        <namespace>/robot</namespace>
        <remapping>~/out:=foot_contact</remapping>
      </ros>
      <frame_name>foot_link</frame_name>
    </plugin>
  </sensor>
</gazebo>
```

### Force/Torque Sensor

```xml
<gazebo reference="wrist_link">
  <sensor name="force_torque" type="force_torque">
    <update_rate>100</update_rate>
    <force_torque>
      <frame>child</frame>
      <measure_direction>child_to_parent</measure_direction>
    </force_torque>
    <plugin name="ft_sensor_plugin" filename="libgazebo_ros_ft_sensor.so">
      <ros>
        <namespace>/robot</namespace>
        <remapping>~/out:=wrist_ft</remapping>
      </ros>
    </plugin>
  </sensor>
</gazebo>
```

## Sensor Fusion

### Combining Multiple Sensors

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, LaserScan
from geometry_msgs.msg import PoseStamped
import numpy as np

class SensorFusion(Node):
    def __init__(self):
        super().__init__('sensor_fusion')
        
        # Subscribe to multiple sensors
        self.imu_sub = self.create_subscription(Imu, '/imu', self.imu_callback, 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        
        # Publisher for fused estimate
        self.pose_pub = self.create_publisher(PoseStamped, '/fused_pose', 10)
        
        self.imu_data = None
        self.scan_data = None
        
    def imu_callback(self, msg):
        self.imu_data = msg
        self.fuse_data()
        
    def scan_callback(self, msg):
        self.scan_data = msg
        self.fuse_data()
        
    def fuse_data(self):
        if self.imu_data is None or self.scan_data is None:
            return
        
        # Simple fusion example
        # In practice, use Kalman filter or particle filter
        
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = 'map'
        
        # Use IMU for orientation
        pose.pose.orientation = self.imu_data.orientation
        
        # Use LIDAR for position (simplified)
        # In reality, you'd use SLAM
        
        self.pose_pub.publish(pose)
```

## Sensor Noise Modeling

### Realistic Noise

```python
import numpy as np

class SensorNoiseModel:
    @staticmethod
    def add_gaussian_noise(data, mean=0.0, std=0.01):
        """Add Gaussian noise to sensor data"""
        noise = np.random.normal(mean, std, data.shape)
        return data + noise
    
    @staticmethod
    def add_outliers(data, outlier_prob=0.01, outlier_range=10.0):
        """Add random outliers to simulate sensor failures"""
        mask = np.random.random(data.shape) < outlier_prob
        outliers = np.random.uniform(-outlier_range, outlier_range, data.shape)
        return np.where(mask, outliers, data)
    
    @staticmethod
    def add_drift(data, drift_rate=0.001):
        """Add sensor drift over time"""
        drift = np.cumsum(np.ones_like(data) * drift_rate)
        return data + drift
```

## Best Practices

1. **Match Real Sensor Specs** - Use manufacturer datasheets
2. **Add Realistic Noise** - Don't use perfect sensors
3. **Consider Update Rates** - Match real sensor frequencies
4. **Test Edge Cases** - Simulate sensor failures
5. **Validate with Real Data** - Compare sim vs real

## Next Steps

You've completed Module 2! Move on to [Module 3: The AI-Robot Brain](../module3/overview.md) to learn about NVIDIA Isaac.
